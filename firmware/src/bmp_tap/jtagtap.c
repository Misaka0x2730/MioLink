/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022-2023 1BitSquared <info@1bitsquared.com>
 * Modified by Rachel Mant <git@dragonmux.network>
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the low-level JTAG TAP interface.  */

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "platform.h"
#include "target_serial.h"

#include "tap_pio.h"
#include "jtagtap.h"
#include "adiv5.h"

#include "pio_jtag.pio.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define TARGET_JTAG_TICKS_NO_FINAL(ticks) ((ticks) - 1) /**< Encode \a ticks for PIO sequences without a final TMS bit. */
#define TARGET_JTAG_TICKS_FINAL(ticks)    ((ticks) - 2) /**< Encode \a ticks for PIO sequences with a final TMS bit. */

/**
 * \brief Maximum TMS tick count accepted by \ref jtagtap_tms_seq.
 *
 * Bounded by the width of the \c tms_states parameter (\c uint32_t), which holds the TMS bit
 * pattern packed LSB-first. Independent of the PIO FIFO word width even though both happen to
 * be 32 on RP2040.
 */
#define JTAG_TMS_SEQ_MAX_TICKS (32U)

/**
 * \brief Number of bits per byte used when packing \c tms_states into the PIO command buffer.
 */
#define JTAG_BITS_PER_BYTE (8U)

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Initial TMS/TDI level encoded in the PIO command stream.
 */
typedef enum {
    TARGET_JTAG_SET_INITIAL_0 = 0, /**< Drive the controlled line low for the initial bit. */
    TARGET_JTAG_SET_INITIAL_1 = 1, /**< Drive the controlled line high for the initial bit. */
} target_jtag_set_initial_t;

/**
 * \brief Final TMS/TDI level encoded in the PIO command stream.
 */
typedef enum {
    TARGET_JTAG_SET_FINAL_NO = 0, /**< Do not change the line at the final tick. */
    TARGET_JTAG_SET_FINAL_0 = 1,  /**< Drive the controlled line low for the final tick. */
    TARGET_JTAG_SET_FINAL_1 = 2,  /**< Drive the controlled line high for the final tick. */
} target_jtag_set_final_t;

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

jtag_proc_s jtag_proc = {0}; /**< Black Magic JTAG procedure vtable, populated by \c jtagtap_init. */

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Soft-reset the TAP via a TMS-high sequence (delegates to \c jtagtap_soft_reset).
 */
static void jtagtap_reset(void);

/**
 * \brief Shift \a ticks bits of \a tms_states out on TMS to drive the TAP state machine.
 *
 * \param[in] tms_states Packed TMS bit pattern (LSB first).
 * \param[in] ticks      Number of bits to shift.
 */
static void jtagtap_tms_seq(uint32_t tms_states, size_t ticks);

/**
 * \brief Bidirectional TDI/TDO shift of \a clock_cycles bits with optional final TMS=1.
 *
 * \param[out] data_out      Destination for shifted-in TDO bits.
 * \param[in]  final_tms     If \c true, assert TMS on the last tick to exit Shift state.
 * \param[in]  data_in       Source TDI bits (LSB-first byte stream).
 * \param[in]  clock_cycles  Number of bits to shift.
 */
static void jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t clock_cycles);

/**
 * \brief Like \c jtagtap_tdi_tdo_seq but discards the TDO data.
 *
 * \param[in] final_tms     If \c true, assert TMS on the last tick.
 * \param[in] data_in       Source TDI bits.
 * \param[in] clock_cycles  Number of bits to shift.
 */
static void jtagtap_tdi_seq(bool final_tms, const uint8_t *data_in, size_t clock_cycles);

/**
 * \brief Step one TAP clock with the given TMS / TDI levels and sample TDO.
 *
 * \param[in] tms TMS level.
 * \param[in] tdi TDI level.
 * \return Sampled TDO bit.
 */
static bool jtagtap_next(bool tms, bool tdi);

/**
 * \brief Run \a clock_cycles idle clocks with fixed TMS / TDI.
 *
 * \param[in] tms          TMS level held for the run.
 * \param[in] tdi          TDI level held for the run.
 * \param[in] clock_cycles Number of clocks to run.
 */
static void jtagtap_cycle(bool tms, bool tdi, size_t clock_cycles);

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Configure the JTAG PIO program/state machines and populate the \c jtag_proc vtable.
 */
void jtagtap_init(void)
{
    /* Snapshot the in-use interface frequency before pio_sm_init reprograms the SM's clkdiv
     * register from the default config (which would otherwise reset the divider to 1.0).
     * The captured value is reapplied after the SMs are back up. */
    const uint32_t saved_freq = platform_max_frequency_get();

    /* JTAG reuses TDI/TDO, which the target-serial bridge may currently drive as a UART.
     * Force the bridge to drop those pins before we reconfigure them as PIO, otherwise the
     * UART driver would keep stale ownership of GPIOs we are about to repurpose. */
    target_serial_tap_acquire_tdi_tdo();

    tap_pio_disable_all_machines(TAP_PIO_SWD);
    tap_pio_disable_all_machines(TAP_PIO_JTAG);

    const platform_target_pins_t *target_pins = platform_get_target_pins();

    if (target_pins->tms_dir != PIN_NOT_CONNECTED) {
        gpio_init(target_pins->tms_dir);
        gpio_set_dir(target_pins->tms_dir, GPIO_OUT);
        gpio_put(target_pins->tms_dir, true);
    }

    gpio_init(target_pins->tck);
    pio_gpio_init(TAP_PIO_JTAG, target_pins->tck);

    gpio_init(target_pins->tdo);
    gpio_set_pulls(target_pins->tdo, true, false);
    pio_gpio_init(TAP_PIO_JTAG, target_pins->tdo);

    gpio_init(target_pins->tdi);
    pio_gpio_init(TAP_PIO_JTAG, target_pins->tdi);

    gpio_init(target_pins->tms);
    pio_gpio_init(TAP_PIO_JTAG, target_pins->tms);

    pio_clear_instruction_memory(TAP_PIO_JTAG);

    const uint32_t pindirs_value = (1U << target_pins->tck) | (1U << target_pins->tdi) | (1U << target_pins->tms);
    const uint32_t pindirs_mask = (1U << target_pins->tck) | (1U << target_pins->tdi) | (1U << target_pins->tdo) |
        (1U << target_pins->tms);
    const uint32_t pins_value = 0;
    const uint32_t pins_mask = (1U << target_pins->tck) | (1U << target_pins->tdi) | (1U << target_pins->tms);

    pio_sm_set_pindirs_with_mask(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pindirs_value, pindirs_mask);
    pio_sm_set_pins_with_mask(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pins_value, pins_mask);

    tap_pio_disable_input_sync(TAP_PIO_JTAG, target_pins->tdo);

    pio_add_program_at_offset(TAP_PIO_JTAG, &target_jtag_program, target_jtag_program.origin);

    pio_sm_config prog_config = target_jtag_program_get_default_config(0);

    /* JTAG TDI/TDO sequence SM */
    sm_config_set_in_pins(&prog_config, target_pins->tdo);
    sm_config_set_out_pins(&prog_config, target_pins->tdi, 1);
    sm_config_set_sideset_pins(&prog_config, target_pins->tck);
    sm_config_set_set_pins(&prog_config, target_pins->tms, 1);
    sm_config_set_out_shift(&prog_config, true, true, JTAG_BITS_PER_BYTE);
    sm_config_set_in_shift(&prog_config, true, true, JTAG_BITS_PER_BYTE);

    pio_sm_init(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, target_jtag_program.origin, &prog_config);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, false);

    /* JTAG TMS sequence SM */
    sm_config_set_in_pins(&prog_config, target_pins->tdo);
    sm_config_set_out_pins(&prog_config, target_pins->tms, 1);
    sm_config_set_sideset_pins(&prog_config, target_pins->tck);
    sm_config_set_set_pins(&prog_config, target_pins->tdi, 1);
    sm_config_set_out_shift(&prog_config, true, true, JTAG_BITS_PER_BYTE);
    sm_config_set_in_shift(&prog_config, true, true, JTAG_BITS_PER_BYTE);

    pio_sm_init(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ, target_jtag_program.origin, &prog_config);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ, false);

    platform_max_frequency_set(saved_freq);

    jtag_proc.jtagtap_reset = jtagtap_reset;
    jtag_proc.jtagtap_next = jtagtap_next;
    jtag_proc.jtagtap_tms_seq = jtagtap_tms_seq;
    jtag_proc.jtagtap_tdi_tdo_seq = jtagtap_tdi_tdo_seq;
    jtag_proc.jtagtap_tdi_seq = jtagtap_tdi_seq;
    jtag_proc.jtagtap_cycle = jtagtap_cycle;
    jtag_proc.tap_idle_cycles = 1;

    /* Ensure we're in JTAG mode. Start with a complete SWD reset of at least 50 cycles. */
    jtagtap_cycle(true, false, 51U);
    /* Try the deprecated 16-bit SWD-to-JTAG select sequence first. */
    jtagtap_tms_seq(ADIV5_SWD_TO_JTAG_SELECT_SEQUENCE, 16U);
    /* Another full SWD reset to complete the legacy path. */
    jtagtap_cycle(true, false, 51U);
    /*
     * Parts that do not implement the deprecated sequence require the
     * SWD-to-Dormant transition followed by the 128-bit Selection Alert
     * and the ARM JTAG-DP activation code to leave dormant state.
     */
    jtagtap_tms_seq(ADIV5_SWD_TO_DORMANT_SEQUENCE, 16U);
    jtagtap_tms_seq(0xffU, 8U); /* 8 reset cycles to ensure target is in a happy place */
    jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_0, 32U);
    jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_1, 32U);
    jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_2, 32U);
    jtagtap_tms_seq(ADIV5_SELECTION_ALERT_SEQUENCE_3, 32U);
    /*
     * 4 SWDIOTMS LOW cycles followed by the 8-bit ARM JTAG-DP activation code,
     * combined into a single 12-bit shift (activation code shifted left by 4).
     */
    jtagtap_tms_seq(ADIV5_ACTIVATION_CODE_ARM_JTAG_DP << 4U, 12U);
}

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static void jtagtap_reset(void)
{
    jtagtap_soft_reset();
}

static bool jtagtap_next(const bool tms, const bool tdi)
{
    uint8_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, true);

    pio_buffer[data_amount++] = TARGET_JTAG_TICKS_NO_FINAL(1);
    pio_buffer[data_amount++] = (tms ? TARGET_JTAG_SET_INITIAL_1 : TARGET_JTAG_SET_INITIAL_0);
    pio_buffer[data_amount++] = TARGET_JTAG_SET_FINAL_NO;
    pio_buffer[data_amount++] = (tdi ? 1 : 0);

    tap_pio_dma_send_uint8(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, data_amount);
    const bool result = (pio_sm_get_blocking(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ) != 0);

    pio_sm_clear_fifos(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, false);

    return result;
}

static void jtagtap_tms_seq(const uint32_t tms_states, const size_t ticks)
{
    assert((ticks > 0) && (ticks <= JTAG_TMS_SEQ_MAX_TICKS));

    uint8_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ, true);

    pio_buffer[data_amount++] = TARGET_JTAG_TICKS_NO_FINAL(ticks);
    pio_buffer[data_amount++] = TARGET_JTAG_SET_INITIAL_1;
    pio_buffer[data_amount++] = TARGET_JTAG_SET_FINAL_NO;

    size_t data_bytes = ticks / JTAG_BITS_PER_BYTE;
    if (ticks % JTAG_BITS_PER_BYTE) {
        data_bytes++;
    }

    for (uint8_t i = 0; (i < data_bytes); i++) {
        pio_buffer[data_amount++] = ((tms_states >> (JTAG_BITS_PER_BYTE * i)) & 0xFF);
    }

    tap_pio_dma_send_recv_uint8(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ, pio_buffer, NULL, data_amount, data_bytes);
    tap_pio_wait_for_tx_stall(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ);

    pio_sm_clear_fifos(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TMS_SEQ, false);
}

static void jtagtap_tdi_tdo_seq(
    uint8_t *const data_out, const bool final_tms, const uint8_t *const data_in, size_t clock_cycles)
{
    uint8_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    if (clock_cycles == 0) {
        return;
    }

    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, true);

    if (clock_cycles == 1) {
        pio_buffer[data_amount++] = TARGET_JTAG_TICKS_NO_FINAL(1);
        pio_buffer[data_amount++] = (final_tms ? TARGET_JTAG_SET_INITIAL_1 : TARGET_JTAG_SET_INITIAL_0);
        pio_buffer[data_amount++] = TARGET_JTAG_SET_FINAL_NO;
        pio_buffer[data_amount++] = data_in[0];

        tap_pio_dma_send_recv_uint8(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, data_out, data_amount, 1);
        data_out[0] >>= (JTAG_BITS_PER_BYTE - 1);
    } else {
        pio_buffer[data_amount++] = TARGET_JTAG_TICKS_FINAL(clock_cycles);
        pio_buffer[data_amount++] = TARGET_JTAG_SET_INITIAL_0;
        pio_buffer[data_amount++] = (final_tms ? TARGET_JTAG_SET_FINAL_1 : TARGET_JTAG_SET_FINAL_0);

        size_t data_bytes = clock_cycles / JTAG_BITS_PER_BYTE;
        if (clock_cycles % JTAG_BITS_PER_BYTE) {
            data_bytes++;
        }

        assert((data_amount + data_bytes) <= TAP_PIO_DMA_BUF_SIZE);
        memcpy(&(pio_buffer[data_amount]), data_in, data_bytes);

        data_amount += data_bytes;

        size_t data_out_cnt = tap_pio_dma_send_recv_uint8(
            TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, data_out, data_amount, data_bytes);

        /* tap_pio_dma_send_recv_uint8 keeps draining the RX FIFO while the TX DMA is busy and
         * may return more reads than data_bytes; surplus bytes are discarded but counted. Clamp
         * to data_bytes so the final-byte shift below does not index past data_out. */
        if (data_out_cnt > data_bytes) {
            data_out_cnt = data_bytes;
        }

        if ((clock_cycles % JTAG_BITS_PER_BYTE) != 0) {
            data_out[data_out_cnt - 1] >>= (JTAG_BITS_PER_BYTE - (clock_cycles % JTAG_BITS_PER_BYTE));
        }
    }

    tap_pio_wait_for_tx_stall(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);

    pio_sm_clear_fifos(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, false);
}

static void jtagtap_tdi_seq(const bool final_tms, const uint8_t *const data_in, const size_t clock_cycles)
{
    uint8_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    if (clock_cycles == 0) {
        return;
    }

    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, true);

    if (clock_cycles == 1) {
        pio_buffer[data_amount++] = TARGET_JTAG_TICKS_NO_FINAL(1);
        pio_buffer[data_amount++] = (final_tms ? TARGET_JTAG_SET_INITIAL_1 : TARGET_JTAG_SET_INITIAL_0);
        pio_buffer[data_amount++] = TARGET_JTAG_SET_FINAL_NO;
        pio_buffer[data_amount++] = data_in[0];

        tap_pio_dma_send_recv_uint8(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, NULL, data_amount, 1);
    } else {
        pio_buffer[data_amount++] = TARGET_JTAG_TICKS_FINAL(clock_cycles);
        pio_buffer[data_amount++] = TARGET_JTAG_SET_INITIAL_0;
        pio_buffer[data_amount++] = (final_tms ? TARGET_JTAG_SET_FINAL_1 : TARGET_JTAG_SET_FINAL_0);

        size_t data_bytes = clock_cycles / JTAG_BITS_PER_BYTE;
        if (clock_cycles % JTAG_BITS_PER_BYTE) {
            data_bytes++;
        }

        assert((data_amount + data_bytes) <= TAP_PIO_DMA_BUF_SIZE);
        memcpy(&(pio_buffer[data_amount]), data_in, data_bytes);

        data_amount += data_bytes;

        tap_pio_dma_send_recv_uint8(
            TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, NULL, data_amount, data_bytes);
    }

    tap_pio_wait_for_tx_stall(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);

    pio_sm_clear_fifos(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, false);
}

static void jtagtap_cycle(const bool tms, const bool tdi, const size_t clock_cycles)
{
    uint8_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, true);

    pio_buffer[data_amount++] = TARGET_JTAG_TICKS_NO_FINAL(clock_cycles);
    pio_buffer[data_amount++] = (tms ? TARGET_JTAG_SET_INITIAL_1 : TARGET_JTAG_SET_INITIAL_0);
    pio_buffer[data_amount++] = TARGET_JTAG_SET_FINAL_NO;

    size_t data_bytes = clock_cycles / JTAG_BITS_PER_BYTE;
    if (clock_cycles % JTAG_BITS_PER_BYTE) {
        data_bytes++;
    }

    assert((data_amount + data_bytes) <= TAP_PIO_DMA_BUF_SIZE);
    memset(&(pio_buffer[data_amount]), (tdi ? 0xFF : 0), data_bytes);

    data_amount += data_bytes;

    tap_pio_dma_send_recv_uint8(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, pio_buffer, NULL, data_amount, data_bytes);

    tap_pio_wait_for_tx_stall(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);

    pio_sm_clear_fifos(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ);
    pio_sm_set_enabled(TAP_PIO_JTAG, TAP_PIO_SM_JTAG_TDI_TDO_SEQ, false);
}