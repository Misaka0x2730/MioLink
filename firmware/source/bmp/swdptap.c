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

#include "tap_pio_common.h"

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

static swdio_status_t tms_dir = SWDIO_STATUS_FLOAT;

extern uint32_t target_interface_frequency;

void swdptap_init(void)
{
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, false);
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);
    pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, false);
    pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, false);
    pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, false);
    pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, false);

    gpio_init(TARGET_TMS_DIR_PIN);
    gpio_init(TARGET_TCK_PIN);
    gpio_init(TARGET_TMS_PIN);

    gpio_set_pulls(TARGET_TMS_PIN, true, false);

    pio_gpio_init(TARGET_SWD_PIO, TARGET_TMS_DIR_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TCK_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TMS_PIN);

	pio_clear_instruction_memory(TARGET_SWD_PIO);

    pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN), (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));
    pio_sm_set_pins_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 0, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));

    tap_pio_common_disable_input_sync(TARGET_SWD_PIO, TARGET_TMS_PIN);

    pio_add_program_at_offset(TARGET_SWD_PIO, &target_swd_program, 0);

    pio_sm_config prog_config = target_swd_program_get_default_config(0);
    sm_config_set_out_pins(&prog_config, TARGET_TMS_PIN, 1);
    sm_config_set_in_pins(&prog_config, TARGET_TMS_PIN);
    sm_config_set_sideset_pins(&prog_config, TARGET_TCK_PIN);
    sm_config_set_set_pins(&prog_config, TARGET_TMS_DIR_PIN, 2);
    sm_config_set_out_shift(&prog_config, true, true, 32);
    sm_config_set_in_shift(&prog_config, true, true, 32);

    pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 0, &prog_config);
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, true);

    sm_config_set_out_shift(&prog_config, true, true, 8);
    sm_config_set_in_shift(&prog_config, true, true, 3);
    pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, 0, &prog_config);
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);

    platform_max_frequency_set(target_interface_frequency);

    swd_proc.seq_in = swdptap_seq_in;
	swd_proc.seq_in_parity = swdptap_seq_in_parity;
	swd_proc.seq_out = swdptap_seq_out;
	swd_proc.seq_out_parity = swdptap_seq_out_parity;
}

static uint32_t __not_in_flash_func(swdptap_seq_in)(size_t clock_cycles)
{
    if (tms_dir == SWDIO_STATUS_DRIVE)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_IN_TURNAROUND_OUT_TO_IN_POS);
        tms_dir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_IN_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);

    tap_pio_common_wait_for_rx_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ,1);
    const uint32_t value = (tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) >> (32U - clock_cycles));

    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
    tap_pio_common_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

    return value;
}

static bool __not_in_flash_func(swdptap_seq_in_parity)(uint32_t *ret, size_t clock_cycles)
{
    if (tms_dir == SWDIO_STATUS_DRIVE)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_IN_TURNAROUND_OUT_TO_IN_POS);
        tms_dir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_IN_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles);
    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_TURNAROUND_IN_TO_OUT_POS);

    tap_pio_common_wait_for_rx_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ,1);

    const uint32_t value = (tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) >> (32U - clock_cycles));

    bool bit = false;
    if (clock_cycles == 32)
    {
        tap_pio_common_wait_for_rx_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ,1);
        bit = (tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) != 0);
    }
    else
    {
        bit = ((value & (1u << clock_cycles)) != 0);
    }

    tms_dir = SWDIO_STATUS_DRIVE;
    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
    tap_pio_common_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

    const bool parity = calculate_odd_parity(value);
    *ret = value;
    return parity == bit;
}

static void __not_in_flash_func(swdptap_seq_out)(const uint32_t tms_states, const size_t clock_cycles)
{
    if (tms_dir == SWDIO_STATUS_FLOAT)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS);
        tms_dir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);
    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states);

    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}

static void __not_in_flash_func(swdptap_seq_out_parity)(const uint32_t tms_states, const size_t clock_cycles)
{
    const bool parity = calculate_odd_parity(tms_states);

    if (tms_dir == SWDIO_STATUS_FLOAT)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS);
        tms_dir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles);

    if (clock_cycles <= 31)
    {
        uint32_t value = tms_states;
        value |= (parity ? (1u << clock_cycles) : 0);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, value);
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, parity ? (1 << 0) : 0);
    }

    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}

uint8_t __not_in_flash_func(rp2040_pio_adiv5_swd_raw_access_req)(const uint8_t request)
{
    tap_pio_common_disable_sm(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
    tap_pio_common_enable_sm(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);

    if (tms_dir == SWDIO_STATUS_FLOAT)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, SWD_ADIV5_REQ_TURNAROUND_IN_TO_OUT_POS);
        tms_dir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5,SWD_ADIV5_REQ_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, request);
    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);

    tms_dir = SWDIO_STATUS_FLOAT;
    const uint8_t ack = (uint8_t)((tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5) >> 29) & 0x7);

    tap_pio_common_disable_sm(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);
    tap_pio_common_enable_sm(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

    return ack;
}

bool __not_in_flash_func(rp2040_pio_adiv5_swd_raw_access_data)(const uint32_t data_out, uint32_t *data_in, const uint8_t rnw)
{
    uint32_t parity = false;

    if (rnw)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_IN_POS);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 32);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, TARGET_SWD_IDLE_CYCLES - 1);

        tap_pio_common_wait_for_rx_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ,2);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 0);

        *data_in = tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
        parity = (tap_pio_common_read_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) != 0) == calculate_odd_parity(*data_in);

        tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
        tap_pio_common_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
    }
    else
    {
        parity = calculate_odd_parity(data_out);

        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 32 + TARGET_SWD_IDLE_CYCLES);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, data_out);
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, (parity ? (1 << 0) : 0));

        tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
    }

    tms_dir = SWDIO_STATUS_DRIVE;

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
    if (tms_dir == SWDIO_STATUS_FLOAT)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_TURNAROUND_IN_TO_OUT_POS);
        tms_dir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, SWD_SEQ_OUT_POS);
    }

    tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);

    uint32_t data_count = clock_cycles / 32;
    if ((clock_cycles % 32) != 0)
    {
        data_count++;
    }

    for (size_t i = 0; i < data_count; i++)
    {
        tap_pio_common_push_data(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states[i]);
    }

    tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}