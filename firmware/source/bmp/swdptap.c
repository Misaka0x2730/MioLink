/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

#include "general.h"
#include "platform.h"
#include "swd.h"
#include "maths_utils.h"

#include "hardware/pio.h"
#include "target_swd.pio.h"

#if !defined(SWDIO_IN_PORT)
#define SWDIO_IN_PORT SWDIO_PORT
#endif
#if !defined(SWDIO_IN_PIN)
#define SWDIO_IN_PIN SWDIO_PIN
#endif

typedef enum swdio_status_e {
	SWDIO_STATUS_FLOAT = 0,
	SWDIO_STATUS_DRIVE
} swdio_status_t;

swd_proc_s swd_proc;

static inline void swdtap_clear_fifos(void) __attribute__((always_inline));
static inline bool swdtap_check_if_not_stalled(void) __attribute__((always_inline));
static inline void swdtap_wait_for_tx_stall(void) __attribute__((always_inline));
static inline void swdtap_wait_for_rx_level(const uint8_t level) __attribute__((always_inline));
static inline void swdtap_set_push_pull_thresholds(const uint8_t push, const uint8_t pull) __attribute__((always_inline));
static uint32_t swdptap_seq_in(size_t clock_cycles) __attribute__((optimize(3)));
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));

#define SWD_START_PROGRAM_POS                  (0)
#define SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS   (1)
#define SWD_SEQ_OUT_POS                        (3)
#define SWD_SEQ_IN_TURNAROUND_OUT_TO_IN_POS    (8)
#define SWD_SEQ_IN_POS                         (10)
#define SWD_ADIV5_REQ_TURNAROUND_IN_TO_OUT_POS (15)
#define SWD_ADIV5_REQ_POS                      (17)
#define SWD_TURNAROUND_IN_TO_OUT_POS           (26)

static swdio_status_t olddir = SWDIO_STATUS_FLOAT;

extern uint32_t target_interface_frequency;

void swdptap_init(void)
{
    gpio_init(TARGET_TMS_DIR_PIN);
    gpio_init(TARGET_TCK_PIN);
    gpio_init(TARGET_TMS_PIN);

    gpio_set_pulls(TARGET_TMS_PIN, true, false);

    pio_gpio_init(TARGET_SWD_PIO, TARGET_TMS_DIR_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TCK_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TMS_PIN);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, false);
	pio_clear_instruction_memory(TARGET_SWD_PIO);

    pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN), (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));
    pio_sm_set_pins_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, 0, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));

    pio_add_program_at_offset(TARGET_SWD_PIO, &target_non_iso_swd_program, 0);
    pio_sm_config prog_config = target_non_iso_swd_program_get_default_config(0);
    sm_config_set_out_pins(&prog_config, TARGET_TMS_PIN, 1);
    sm_config_set_in_pins(&prog_config, TARGET_TMS_PIN);
    sm_config_set_sideset_pins(&prog_config, TARGET_TCK_PIN);
    sm_config_set_set_pins(&prog_config, TARGET_TMS_DIR_PIN, 2);
    sm_config_set_out_shift(&prog_config, true, true, 0);
    sm_config_set_in_shift(&prog_config, true, true, 0);

    TARGET_SWD_PIO->input_sync_bypass |= (1u << TARGET_TMS_PIN);

    pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, 0, &prog_config);
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, true);

    platform_max_frequency_set(target_interface_frequency);

    swd_proc.seq_in = swdptap_seq_in;
	swd_proc.seq_in_parity = swdptap_seq_in_parity;
	swd_proc.seq_out = swdptap_seq_out;
	swd_proc.seq_out_parity = swdptap_seq_out_parity;
}

static void swdtap_clear_fifos(void)
{
    hw_xor_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

static bool swdtap_check_if_not_stalled(void)
{
    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    return ((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);
}

static void swdtap_wait_for_tx_stall(void)
{
    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);
}

static void swdtap_wait_for_rx_level(const uint8_t level)
{
    if ((level > 0) && (level <= 8))
    {
        while (((TARGET_SWD_PIO->flevel & PIO_FLEVEL_RX0_BITS) >> PIO_FLEVEL_RX0_LSB) < level);
    }
}

static void swdtap_set_push_pull_thresholds(const uint8_t push, const uint8_t pull)
{
    hw_write_masked(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl,
                    ((uint32_t)(push) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
                    ((uint32_t)(pull) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB),
                    PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);
}

static uint32_t __not_in_flash_func(swdptap_seq_in)(size_t clock_cycles)
{
    if (olddir == SWDIO_STATUS_DRIVE)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_IN_TURNAROUND_OUT_TO_IN_POS;
        olddir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_IN_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles - 1;
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_START_PROGRAM_POS;

    swdtap_wait_for_rx_level(1);
    const uint32_t value = (TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> (32U - clock_cycles));

    swdtap_wait_for_tx_stall();
    swdtap_clear_fifos();

    return value;
}

static bool __not_in_flash_func(swdptap_seq_in_parity)(uint32_t *ret, size_t clock_cycles)
{
    if (olddir == SWDIO_STATUS_DRIVE)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_IN_TURNAROUND_OUT_TO_IN_POS;
        olddir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_IN_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles;
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_TURNAROUND_IN_TO_OUT_POS;

    swdtap_wait_for_rx_level(1);

    const uint32_t value = (TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> (32U - clock_cycles));

    bool bit = false;
    if (clock_cycles == 32)
    {
        swdtap_wait_for_rx_level(1);
        bit = (TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] != 0);
    }
    else
    {
        bit = ((value & (1u << clock_cycles)) != 0);
    }

    olddir = SWDIO_STATUS_DRIVE;
    swdtap_wait_for_tx_stall();
    swdtap_clear_fifos();

    const bool parity = calculate_odd_parity(value);
    *ret = value;
    return parity == bit;
}

static void __not_in_flash_func(swdptap_seq_out)(const uint32_t tms_states, const size_t clock_cycles)
{
    if (olddir == SWDIO_STATUS_FLOAT)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS;
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles - 1;
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = tms_states;

    if (clock_cycles == 32)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;
    }

    swdtap_wait_for_tx_stall();
}

static void __not_in_flash_func(swdptap_seq_out_parity)(const uint32_t tms_states, const size_t clock_cycles)
{
    const bool parity = calculate_odd_parity(tms_states);

    if (olddir == SWDIO_STATUS_FLOAT)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS;
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles;

    if (clock_cycles <= 31)
    {
        uint32_t value = tms_states;
        value |= (parity ? (1u << clock_cycles) : 0);
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = value;

        if (clock_cycles == 31)
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;
        }
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = tms_states;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = parity;
    }

    swdtap_wait_for_tx_stall();
}

uint8_t __not_in_flash_func(rp2040_pio_adiv5_swd_raw_access_req)(const uint8_t request)
{
    swdtap_set_push_pull_thresholds(3, 8);

    if (olddir == SWDIO_STATUS_FLOAT)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_ADIV5_REQ_TURNAROUND_IN_TO_OUT_POS;
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_ADIV5_REQ_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = request;

    swdtap_wait_for_tx_stall();

    olddir = SWDIO_STATUS_FLOAT;
    const uint8_t ack = (uint8_t)((TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> 29) & 0x7);

    swdtap_set_push_pull_thresholds(0, 0);
    return ack;
}

bool __not_in_flash_func(rp2040_pio_adiv5_swd_raw_access_data)(const uint32_t data_out, uint32_t *data_in, const uint8_t rnw)
{
    uint32_t parity = false;

    if (rnw)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_IN_POS;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 32;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = TARGET_SWD_IDLE_CYCLES - 1;

        swdtap_wait_for_rx_level(2);
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;

        *data_in = TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM];
        parity = (TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] != 0) == calculate_odd_parity(*data_in);

        swdtap_wait_for_tx_stall();
        swdtap_clear_fifos();
    }
    else
    {
        const bool parity = calculate_odd_parity(data_out);

        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 32 + TARGET_SWD_IDLE_CYCLES;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = data_out;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (parity ? (1 << 0) : 0);

        swdtap_wait_for_tx_stall();
    }

    olddir = SWDIO_STATUS_DRIVE;

    return parity;
}

uint8_t __not_in_flash_func(rp2040_pio_adiv5_swd_write_no_check)(const uint8_t request, const uint32_t data)
{
    const uint8_t ack = rp2040_pio_adiv5_swd_raw_access_req(request);
    rp2040_pio_adiv5_swd_raw_access_data(data, NULL, 0);

    return ack;
}

uint8_t __not_in_flash_func(rp2040_pio_adiv5_swd_read_no_check)(const uint8_t request, uint32_t *data)
{
    const uint8_t ack = rp2040_pio_adiv5_swd_raw_access_req(request);
    rp2040_pio_adiv5_swd_raw_access_data(0, data, 1);

    return ack;
}

void __not_in_flash_func(swdptap_seq_out_buffer)(const uint32_t *tms_states, const size_t clock_cycles)
{
    if (olddir == SWDIO_STATUS_FLOAT)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS;
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = SWD_SEQ_OUT_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles - 1;

    uint32_t data_count = clock_cycles / 32;
    if ((clock_cycles % 0x1f) != 0)
    {
        data_count++;
    }

    for (size_t i = 0; i < data_count; i++)
    {
        while((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = tms_states[i];
    }

    if ((clock_cycles % 0x1f) == 0)
    {
        while((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;
    }

    swdtap_wait_for_tx_stall();
}