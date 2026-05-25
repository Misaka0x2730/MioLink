/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* This file implements the SW-DP interface. */

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/pio.h"

#include "platform.h"
#include "platform_swdtap.h"
#include "target_serial.h"

#include "tap_pio.h"
#include "swd.h"
#include "maths_utils.h"

#include "pio_swd_miolink_rev_a.pio.h"
#include "pio_swd_miolink_rev_b.pio.h"
#include "pio_swd_miolink_pico.pio.h"
#include "pio_swd_pico.pio.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define SWDP_ACK_OK (0x01U) /**< Three-bit SWD ACK value indicating OK. */

/**
 * \brief Sideset optional-bit cost in PIO instruction encoding.
 *
 * When a PIO program is declared with \c ".side_set N opt", the SDK API
 * (\c sm_config_set_sideset) expects \c bit_count = N + 1: the extra bit selects whether the
 * instruction carries a sideset value or not. All SWD PIO programs in this module use
 * \c opt sideset, so this constant is added to each board's pin count when configuring the SM.
 */
#define SWD_PIO_SIDESET_OPTIONAL_BIT (1U)

#define SWD_REQUEST_BITS         (8U)  /**< SWD request packet length in bits. */
#define SWD_ACK_BITS             (3U)  /**< SWD acknowledge response length in bits (OK / WAIT / FAULT). */
#define SWD_DATA_BITS            (32U) /**< SWD data phase length in bits (one 32-bit word). */
#define SWD_PARITY_BITS          (1U)  /**< Parity bit count appended after the SWD data phase. */
#define SWD_DATA_AND_PARITY_BITS (SWD_DATA_BITS + SWD_PARITY_BITS) /**< Data + parity tick count. */
#define SWD_IDLE_DATA            (0U)  /**< TMS pattern driven during idle cycles (line held low). */

/**
 * \brief TX FIFO words queued after the ADIv5 ACK check for a read transaction.
 *
 * Encoded as \c N-1 into the \c check_ack PIO program's X register; on ACK mismatch the program
 * discards exactly this many TX FIFO words to keep the bus state consistent.
 */
#define SWD_ADIV5_READ_DATA_PHASE_WORDS (5U)

/**
 * \brief TX FIFO words queued after the ADIv5 ACK check for a write transaction.
 *
 * Encoded as \c N-1 into the \c check_ack PIO program's X register; on ACK mismatch the program
 * discards exactly this many TX FIFO words to keep the bus state consistent.
 */
#define SWD_ADIV5_WRITE_DATA_PHASE_WORDS (4U)

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Set of PIO programs implementing the SWD low-level sequences for one board flavour.
 */
typedef struct {
    const struct pio_program *swd_start_prog;                /**< Initial start-up program. */
    const struct pio_program *swd_seq_out_turnaround;        /**< SWD sequence-out path including turnaround. */
    const struct pio_program *swd_seq_out;                   /**< SWD sequence-out without turnaround. */
    const struct pio_program *swd_seq_in_turnaround;         /**< SWD sequence-in path including turnaround. */
    const struct pio_program *swd_seq_in;                    /**< SWD sequence-in without turnaround. */
    const struct pio_program *swd_adiv5_check_ack;           /**< ADIv5 ACK check-and-branch helper program. */
    const struct pio_program *swd_turnaround_float_to_drive; /**< Float→drive SWDIO turnaround. */
} swd_board_program_t;

/**
 * \brief Whether SWDIO is currently driven by the probe or left floating for the target.
 */
typedef enum swdio_status_e {
    SWDIO_STATUS_FLOAT = 0, /**< Probe SWDIO line is high-Z, listening to the target. */
    SWDIO_STATUS_DRIVE      /**< Probe is actively driving SWDIO. */
} swdio_status_t;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief SWD PIO programs for MioLink rev A.
 */
static const swd_board_program_t miolink_rev_a_programs = {
    .swd_start_prog = &miolink_rev_a_swd_start_program,
    .swd_seq_out_turnaround = &miolink_rev_a_swd_seq_out_turnaround_program,
    .swd_seq_out = &miolink_rev_a_swd_seq_out_program,
    .swd_seq_in_turnaround = &miolink_rev_a_swd_seq_in_turnaround_program,
    .swd_seq_in = &miolink_rev_a_swd_seq_in_program,
    .swd_adiv5_check_ack = &miolink_rev_a_swd_adiv5_check_ack_program,
    .swd_turnaround_float_to_drive = &miolink_rev_a_swd_turnaround_float_to_drive_program,
};

/**
 * \brief SWD PIO programs for MioLink rev B.
 */
static const swd_board_program_t miolink_rev_b_programs = {
    .swd_start_prog = &miolink_rev_b_swd_start_program,
    .swd_seq_out_turnaround = &miolink_rev_b_swd_seq_out_turnaround_program,
    .swd_seq_out = &miolink_rev_b_swd_seq_out_program,
    .swd_seq_in_turnaround = &miolink_rev_b_swd_seq_in_turnaround_program,
    .swd_seq_in = &miolink_rev_b_swd_seq_in_program,
    .swd_adiv5_check_ack = &miolink_rev_b_swd_adiv5_check_ack_program,
    .swd_turnaround_float_to_drive = &miolink_rev_b_swd_turnaround_float_to_drive_program,
};

/**
 * \brief SWD PIO programs for MioLink_Pico.
 */
static const swd_board_program_t miolink_pico_programs = {
    .swd_start_prog = &miolink_pico_swd_start_program,
    .swd_seq_out_turnaround = &miolink_pico_swd_seq_out_turnaround_program,
    .swd_seq_out = &miolink_pico_swd_seq_out_program,
    .swd_seq_in_turnaround = &miolink_pico_swd_seq_in_turnaround_program,
    .swd_seq_in = &miolink_pico_swd_seq_in_program,
    .swd_adiv5_check_ack = &miolink_pico_swd_adiv5_check_ack_program,
    .swd_turnaround_float_to_drive = &miolink_pico_swd_turnaround_float_to_drive_program,
};

/**
 * \brief SWD PIO programs for Raspberry Pi Pico / Pico W.
 */
static const swd_board_program_t pico_programs = {
    .swd_start_prog = &pico_swd_start_program,
    .swd_seq_out_turnaround = &pico_swd_seq_out_turnaround_program,
    .swd_seq_out = &pico_swd_seq_out_program,
    .swd_seq_in_turnaround = &pico_swd_seq_in_turnaround_program,
    .swd_seq_in = &pico_swd_seq_in_program,
    .swd_adiv5_check_ack = &pico_swd_adiv5_check_ack_program,
    .swd_turnaround_float_to_drive = &pico_swd_turnaround_float_to_drive_program,
};

static swdio_status_t tms_dir = SWDIO_STATUS_FLOAT; /**< Cached SWDIO driver state to elide redundant turnarounds. */

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

swd_proc_s swd_proc = {0}; /**< Black Magic SWD procedure vtable, populated by \c swdptap_init. */

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Resolve the SWD PIO program set for the active board.
 *
 * \return Pointer to the board-specific program table.
 */
static const swd_board_program_t *swdtap_get_board_programs(void);

/**
 * \brief Build a PIO command stream for a single seq-in/seq-out (with optional parity).
 *
 * \param[out] buffer       Destination buffer (must hold at least \ref TAP_PIO_DMA_BUF_SIZE words).
 * \param[in]  clock_cycles Number of bits to shift (0 < cycles ≤ 32).
 * \param[in]  data         Data word (for seq-out only).
 * \param[in]  in           \c true for seq-in, \c false for seq-out.
 * \param[in]  parity       Append parity handling sequence.
 * \return Number of words written to \a buffer.
 */
static uint8_t swdtap_prepare_pio_seq(
    uint32_t *buffer, uint32_t clock_cycles, uint32_t data, bool in, bool parity);

/**
 * \brief SWD bare seq-in helper (no parity) — wired into \c swd_proc.seq_in.
 *
 * \param[in] clock_cycles Number of bits to shift in (≤ 32).
 * \return Received value.
 */
static uint32_t swdptap_seq_in(size_t clock_cycles);

/**
 * \brief SWD seq-in with parity check — wired into \c swd_proc.seq_in_parity.
 *
 * \param[out] ret          Receives the value.
 * \param[in]  clock_cycles Number of bits to shift in (≤ 32).
 * \return \c true if parity matched; \c false otherwise.
 */
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles);

/**
 * \brief SWD seq-out helper (no parity) — wired into \c swd_proc.seq_out.
 *
 * \param[in] tms_states   Value to shift out (≤ 32 bits).
 * \param[in] clock_cycles Number of bits to shift.
 */
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles);

/**
 * \brief SWD seq-out with parity — wired into \c swd_proc.seq_out_parity.
 *
 * \param[in] tms_states   Value to shift out.
 * \param[in] clock_cycles Number of bits to shift.
 */
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles);

/**
 * \brief Build a PIO command stream that performs a complete ADIv5 read/write transaction.
 *
 * \param[out] buffer    Destination buffer.
 * \param[in]  request   ADIv5 packet request byte.
 * \param[in]  data      Word to write (write transactions only).
 * \param[in]  rnw       \c true for read, \c false for write.
 * \param[in]  check_ack Insert the ACK-check program after the request.
 * \return Number of words written to \a buffer.
 */
static uint8_t swdtap_adiv5_prepare_pio_seq(
    uint32_t *buffer, uint8_t request, uint32_t data, bool rnw, bool check_ack);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static const swd_board_program_t *swdtap_get_board_programs(void)
{
    const swd_board_program_t *p_board_program = NULL;
    const platform_device_type_t device_type = platform_hwtype();
    assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

    switch (device_type) {
    case PLATFORM_DEVICE_TYPE_MIOLINK:
        if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
            p_board_program = &miolink_rev_a_programs;
        } else {
            p_board_program = &miolink_rev_b_programs;
        }
        break;

    case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
        p_board_program = &miolink_pico_programs;
        break;

    case PLATFORM_DEVICE_TYPE_PICO:
    case PLATFORM_DEVICE_TYPE_PICO_W:
        p_board_program = &pico_programs;
        break;

    default:
        assert(false);
        break;
    }

    return p_board_program;
}

static uint8_t swdtap_prepare_pio_seq(
    uint32_t *buffer, const uint32_t clock_cycles, const uint32_t data, const bool in, const bool parity)
{
    assert(buffer != NULL);
    assert((clock_cycles > 0) && (clock_cycles <= TAP_PIO_MAX_TICKS_PER_TRANSFER));

    const swd_board_program_t *p_board_programs = swdtap_get_board_programs();
    assert(p_board_programs != NULL);

    uint8_t pos = 0;

    if (in) {
        if (tms_dir == SWDIO_STATUS_DRIVE) {
            buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in_turnaround->origin);
            tms_dir = SWDIO_STATUS_FLOAT;
        } else {
            buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in->origin);
        }
    } else {
        if (tms_dir == SWDIO_STATUS_FLOAT) {
            buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
            tms_dir = SWDIO_STATUS_DRIVE;
        } else {
            buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out->origin);
        }
    }

    buffer[pos++] = parity ? clock_cycles : (clock_cycles - 1);

    if (parity) {
        if (in) {
            buffer[pos++] = (uint32_t)(p_board_programs->swd_turnaround_float_to_drive->origin);
            tms_dir = SWDIO_STATUS_DRIVE;
        } else {
            const bool parity_value = (calculate_odd_parity(data) != 0);

            if (clock_cycles < TAP_PIO_MAX_TICKS_PER_TRANSFER) {
                uint32_t data_value = data;
                data_value |= (parity_value ? (1UL << clock_cycles) : 0);
                buffer[pos++] = data_value;
            } else {
                buffer[pos++] = data;
                buffer[pos++] = parity_value ? (1UL << 0) : 0UL;
            }
        }
    } else if (!in) {
        buffer[pos++] = data;
    }

    return pos;
}

static uint32_t swdptap_seq_in(const size_t clock_cycles)
{
    assert((clock_cycles > 0) && (clock_cycles <= TAP_PIO_MAX_TICKS_PER_TRANSFER));

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_prepare_pio_seq(pio_buffer, clock_cycles, 0, true, false);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    const uint32_t value = (pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> (TAP_PIO_MAX_TICKS_PER_TRANSFER - clock_cycles));
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    return value;
}

static bool swdptap_seq_in_parity(uint32_t *ret, const size_t clock_cycles)
{
    assert(ret != NULL);
    assert((clock_cycles > 0) && (clock_cycles <= TAP_PIO_MAX_TICKS_PER_TRANSFER));

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_prepare_pio_seq(pio_buffer, clock_cycles, 0, true, true);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);

    uint32_t value = 0;
    bool parity_read = false;

    if (clock_cycles == TAP_PIO_MAX_TICKS_PER_TRANSFER) {
        value = pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD);
        parity_read = (pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) != 0);
    } else {
        const uint32_t packed = pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> (TAP_PIO_MAX_TICKS_PER_TRANSFER - clock_cycles - 1U);
        parity_read = ((packed & (1UL << clock_cycles)) != 0);
        value = packed & ((1UL << clock_cycles) - 1U);
    }

    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    const bool parity = (calculate_odd_parity(value) != 0);
    *ret = value;
    return parity == parity_read;
}

static void swdptap_seq_out(const uint32_t tms_states, const size_t clock_cycles)
{
    assert((clock_cycles > 0) && (clock_cycles <= TAP_PIO_MAX_TICKS_PER_TRANSFER));

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_prepare_pio_seq(pio_buffer, clock_cycles, tms_states, false, false);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);
}

static void swdptap_seq_out_parity(const uint32_t tms_states, const size_t clock_cycles)
{
    assert((clock_cycles > 0) && (clock_cycles <= TAP_PIO_MAX_TICKS_PER_TRANSFER));

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_prepare_pio_seq(pio_buffer, clock_cycles, tms_states, false, true);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);
}

static uint8_t swdtap_adiv5_prepare_pio_seq(
    uint32_t *buffer, const uint8_t request, const uint32_t data, const bool rnw, const bool check_ack)
{
    assert(buffer != NULL);

    const swd_board_program_t *p_board_programs = swdtap_get_board_programs();
    assert(p_board_programs != NULL);

    uint8_t pos = 0;

    if (tms_dir == SWDIO_STATUS_FLOAT) {
        buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
        tms_dir = SWDIO_STATUS_DRIVE;
    } else {
        buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out->origin);
    }

    buffer[pos++] = SWD_REQUEST_BITS - 1;
    buffer[pos++] = request;
    buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in_turnaround->origin);
    buffer[pos++] = SWD_ACK_BITS - 1;

    if (check_ack) {
        buffer[pos++] = (uint32_t)(p_board_programs->swd_adiv5_check_ack->origin);
        buffer[pos++] = (SWDP_ACK_OK << (SWD_DATA_BITS - SWD_ACK_BITS));

        if (rnw) {
            buffer[pos++] = SWD_ADIV5_READ_DATA_PHASE_WORDS - 1;
        } else {
            buffer[pos++] = SWD_ADIV5_WRITE_DATA_PHASE_WORDS - 1;
        }
    }

    if (rnw) {
        buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in->origin);
        buffer[pos++] = SWD_DATA_AND_PARITY_BITS - 1;
        buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
        buffer[pos++] = TARGET_SWD_IDLE_CYCLES - 1;
        buffer[pos++] = SWD_IDLE_DATA;
    } else {
        buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
        buffer[pos++] = SWD_DATA_AND_PARITY_BITS + TARGET_SWD_IDLE_CYCLES - 1;
        buffer[pos++] = data;
        buffer[pos++] = ((calculate_odd_parity(data) != 0) ? (1 << 0) : 0);
    }

    return pos;
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Configure the SWD PIO program/state machine for the active board and populate \c swd_proc.
 */
void swdptap_init(void)
{
    /* SWD does not use TDI/TDO; release the JTAG-side lockout so the target-serial bridge
     * can rebind TDI/TDO as a UART if the user has previously enabled UART-on-TDI/TDO. */
    target_serial_tap_release_tdi_tdo();

    tap_pio_disable_all_machines(TAP_PIO_SWD);
    tap_pio_disable_all_machines(TAP_PIO_JTAG);

    const platform_target_pins_t *target_pins = platform_get_target_pins();

    uint32_t tms_dir_mask = 0;
    if (target_pins->tms_dir != PIN_NOT_CONNECTED) {
        gpio_init(target_pins->tms_dir);
        pio_gpio_init(TAP_PIO_SWD, target_pins->tms_dir);

        tms_dir_mask = (1U << target_pins->tms_dir);
    }

    gpio_init(target_pins->tck);
    gpio_set_slew_rate(target_pins->tck, GPIO_SLEW_RATE_FAST);
    gpio_init(target_pins->tms);
    gpio_set_slew_rate(target_pins->tms, GPIO_SLEW_RATE_FAST);

    pio_gpio_init(TAP_PIO_SWD, target_pins->tck);
    pio_gpio_init(TAP_PIO_SWD, target_pins->tms);

    pio_sm_set_pindirs_with_mask(TAP_PIO_SWD, TAP_PIO_SM_SWD, tms_dir_mask | (1U << target_pins->tck),
        tms_dir_mask | (1U << target_pins->tck) | (1U << target_pins->tms));
    pio_sm_set_pins_with_mask(
        TAP_PIO_SWD, TAP_PIO_SM_SWD, 0, tms_dir_mask | (1U << target_pins->tck) | (1U << target_pins->tms));

    pio_sm_config swd_program_config = pio_get_default_sm_config();

    const swd_board_program_t *p_board_programs = swdtap_get_board_programs();
    assert(p_board_programs != NULL);

    /* Boards without a level shifter (no TMS_DIR pin) need an internal pull-up on TMS to
     * keep SWDIO defined when the line is left floating between transactions. */
    if (target_pins->tms_dir == PIN_NOT_CONNECTED) {
        gpio_set_pulls(target_pins->tms, true, false);
    }

    tap_pio_disable_input_sync(TAP_PIO_SWD, target_pins->tms);

    pio_clear_instruction_memory(TAP_PIO_SWD);

    pio_add_program_at_offset(TAP_PIO_SWD, p_board_programs->swd_start_prog, p_board_programs->swd_start_prog->origin);
    pio_add_program_at_offset(
        TAP_PIO_SWD, p_board_programs->swd_seq_out_turnaround, p_board_programs->swd_seq_out_turnaround->origin);
    pio_add_program_at_offset(TAP_PIO_SWD, p_board_programs->swd_seq_out, p_board_programs->swd_seq_out->origin);
    pio_add_program_at_offset(
        TAP_PIO_SWD, p_board_programs->swd_seq_in_turnaround, p_board_programs->swd_seq_in_turnaround->origin);
    pio_add_program_at_offset(TAP_PIO_SWD, p_board_programs->swd_seq_in, p_board_programs->swd_seq_in->origin);
    pio_add_program_at_offset(
        TAP_PIO_SWD, p_board_programs->swd_adiv5_check_ack, p_board_programs->swd_adiv5_check_ack->origin);
    pio_add_program_at_offset(TAP_PIO_SWD, p_board_programs->swd_turnaround_float_to_drive,
        p_board_programs->swd_turnaround_float_to_drive->origin);

    sm_config_set_set_pins(
        &swd_program_config, target_pins->swd_pio_set_pin_base, target_pins->swd_pio_set_pin_count);
    sm_config_set_sideset(&swd_program_config,
        target_pins->swd_pio_sideset_pin_count + SWD_PIO_SIDESET_OPTIONAL_BIT, true, false);
    sm_config_set_sideset_pins(&swd_program_config, target_pins->swd_pio_sideset_pin_base);
    sm_config_set_in_pins(&swd_program_config, target_pins->tms);
    sm_config_set_out_pins(&swd_program_config, target_pins->tms, 1);
    sm_config_set_out_shift(&swd_program_config, true, true, 32);
    sm_config_set_in_shift(&swd_program_config, true, true, 32);
    sm_config_set_wrap(&swd_program_config, p_board_programs->swd_start_prog->origin,
        p_board_programs->swd_turnaround_float_to_drive->origin +
            p_board_programs->swd_turnaround_float_to_drive->length - 1);

    pio_sm_init(TAP_PIO_SWD, TAP_PIO_SM_SWD, p_board_programs->swd_start_prog->origin, &swd_program_config);
    pio_sm_set_enabled(TAP_PIO_SWD, TAP_PIO_SM_SWD, true);

    platform_max_frequency_set(platform_max_frequency_get());

    tms_dir = SWDIO_STATUS_FLOAT;

    swd_proc.seq_in = swdptap_seq_in;
    swd_proc.seq_in_parity = swdptap_seq_in_parity;
    swd_proc.seq_out = swdptap_seq_out;
    swd_proc.seq_out_parity = swdptap_seq_out_parity;
}

void swdptap_seq_out_buffer(const uint32_t *tms_states, const size_t clock_cycles)
{
    assert(tms_states != NULL);
    assert(clock_cycles > 0);

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    uint8_t data_amount = 0;

    uint32_t data_count = clock_cycles / TAP_PIO_MAX_TICKS_PER_TRANSFER;
    if ((clock_cycles % TAP_PIO_MAX_TICKS_PER_TRANSFER) != 0) {
        data_count++;
    }

    const swd_board_program_t *p_board_programs = swdtap_get_board_programs();
    assert(p_board_programs != NULL);

    if (tms_dir == SWDIO_STATUS_FLOAT) {
        pio_buffer[data_amount++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
        tms_dir = SWDIO_STATUS_DRIVE;
    } else {
        pio_buffer[data_amount++] = (uint32_t)(p_board_programs->swd_seq_out->origin);
    }

    pio_buffer[data_amount++] = clock_cycles - 1;

    assert((data_amount + data_count) <= TAP_PIO_DMA_BUF_SIZE);
    memcpy(&(pio_buffer[data_amount]), tms_states, data_count * sizeof(pio_buffer[0]));

    data_amount += data_count;

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);
}

uint8_t swdtap_adiv5_write_no_check(const uint8_t request, const uint32_t data)
{
    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_adiv5_prepare_pio_seq(pio_buffer, request, data, false, false);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> 29) & 0x7);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    return ack;
}

uint8_t swdtap_adiv5_read_no_check(const uint8_t request, uint32_t *data)
{
    assert(data != NULL);

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_adiv5_prepare_pio_seq(pio_buffer, request, 0, true, false);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> 29) & 0x7);
    *data = pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD);
    pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    return ack;
}

uint8_t swdtap_adiv5_write_check(const uint8_t request, const uint32_t data)
{
    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_adiv5_prepare_pio_seq(pio_buffer, request, data, false, true);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> 29) & 0x7);
    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    return ack;
}

uint8_t swdtap_adiv5_read_check(const uint8_t request, uint32_t *data, bool *parity)
{
    assert(data != NULL);

    *data = 0U;
    if (parity != NULL) {
        *parity = false;
    }

    uint32_t pio_buffer[TAP_PIO_DMA_BUF_SIZE] = {0};
    const uint8_t data_amount = swdtap_adiv5_prepare_pio_seq(pio_buffer, request, 0, true, true);

    tap_pio_dma_send_uint32(TAP_PIO_SWD, TAP_PIO_SM_SWD, pio_buffer, data_amount);
    const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) >> 29) & 0x7);

    if (ack == SWDP_ACK_OK) {
        *data = pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD);
        const bool parity_value =
            (pio_sm_get_blocking(TAP_PIO_SWD, TAP_PIO_SM_SWD) != 0) == (calculate_odd_parity(*data) != 0);
        if (parity != NULL) {
            *parity = parity_value;
        }
    }

    tap_pio_wait_for_tx_stall(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    pio_sm_clear_fifos(TAP_PIO_SWD, TAP_PIO_SM_SWD);

    return ack;
}
