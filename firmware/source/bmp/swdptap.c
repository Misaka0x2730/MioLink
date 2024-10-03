/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Modified 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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
#include "miolink_revA_target_swd.pio.h"
#include "miolink_revB_target_swd.pio.h"
#include "miolink_pico_target_swd.pio.h"
#include "pico_target_swd.pio.h"

#include "tap_pio_common.h"

static uint32_t swdptap_seq_in(size_t clock_cycles) __attribute__((optimize(3)));
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));

typedef struct {
	uint8_t start;
	uint8_t seq_out_turnaround_in_to_out;
	uint8_t seq_out;
	uint8_t seq_in_turnaround_out_to_in;
	uint8_t seq_in;
	uint8_t adiv5_req_turnaround_in_to_out;
	uint8_t adiv5_req;
	uint8_t turnaround_in_to_out;
} swd_program_pos_t;

const swd_program_pos_t miolink_revA_program_pos = {.start = 0,
	.seq_out_turnaround_in_to_out = 1,
	.seq_out = 3,
	.seq_in_turnaround_out_to_in = 8,
	.seq_in = 10,
	.adiv5_req_turnaround_in_to_out = 15,
	.adiv5_req = 17,
	.turnaround_in_to_out = 26};

const swd_program_pos_t miolink_revB_program_pos = {.start = 0,
	.seq_out_turnaround_in_to_out = 1,
	.seq_out = 2,
	.seq_in_turnaround_out_to_in = 7,
	.seq_in = 8,
	.adiv5_req_turnaround_in_to_out = 13,
	.adiv5_req = 14,
	.turnaround_in_to_out = 22};

const swd_program_pos_t miolink_pico_program_pos = {.start = 0,
	.seq_out_turnaround_in_to_out = 1,
	.seq_out = 2,
	.seq_in_turnaround_out_to_in = 7,
	.seq_in = 8,
	.adiv5_req_turnaround_in_to_out = 13,
	.adiv5_req = 14,
	.turnaround_in_to_out = 22};

const swd_program_pos_t pico_program_pos = {.start = 0,
	.seq_out_turnaround_in_to_out = 1,
	.seq_out = 2,
	.seq_in_turnaround_out_to_in = 7,
	.seq_in = 8,
	.adiv5_req_turnaround_in_to_out = 13,
	.adiv5_req = 14,
	.turnaround_in_to_out = 22};

typedef enum swdio_status_e {
	SWDIO_STATUS_FLOAT = 0,
	SWDIO_STATUS_DRIVE
} swdio_status_t;

swd_proc_s swd_proc;

static swdio_status_t tms_dir = SWDIO_STATUS_FLOAT;

extern uint32_t target_interface_frequency;

static const swd_program_pos_t *swdtap_get_program_pos(void)
{
	const swd_program_pos_t *p_program_pos = NULL;
	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			p_program_pos = &miolink_revA_program_pos;
		} else {
			p_program_pos = &miolink_revB_program_pos;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_program_pos = &miolink_pico_program_pos;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_program_pos = &pico_program_pos;
		break;

	default:
		assert(false);
		break;
	}

	return p_program_pos;
}

void swdptap_init(void)
{
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, false);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, false);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, false);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, false);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, false);

	const platform_target_pins_t *target_pins = platform_get_target_pins();

	uint32_t tms_dir_mask = 0;
	if (target_pins->tms_dir != PIN_NOT_CONNECTED) {
		gpio_init(target_pins->tms_dir);
		pio_gpio_init(TARGET_SWD_PIO, target_pins->tms_dir);

		tms_dir_mask = (1 << target_pins->tms_dir);
	}

	gpio_init(target_pins->tck);
	gpio_init(target_pins->tms);

	pio_gpio_init(TARGET_SWD_PIO, target_pins->tck);
	pio_gpio_init(TARGET_SWD_PIO, target_pins->tms);

	pio_clear_instruction_memory(TARGET_SWD_PIO);

	pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_dir_mask | (1 << target_pins->tck),
		tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));
	pio_sm_set_pins_with_mask(
		TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 0, tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));

	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	const pio_program_t *program = NULL;
	pio_sm_config prog_config = {0};
	uint8_t set_pins_base = PIN_NOT_CONNECTED;
	uint8_t set_pins_count = 1;
	uint8_t sideset_pins_base = PIN_NOT_CONNECTED;

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			program = &miolink_revA_target_swd_program;
			prog_config = miolink_revA_target_swd_program_get_default_config(0);
			set_pins_base = target_pins->tms;
			set_pins_count = 2;
			sideset_pins_base = target_pins->tck;
		} else {
			program = &miolink_revB_target_swd_program;
			prog_config = miolink_revB_target_swd_program_get_default_config(0);
			set_pins_base = target_pins->tms;
			set_pins_count = 1;
			sideset_pins_base = target_pins->tck;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		program = &miolink_pico_target_swd_program;
		prog_config = miolink_pico_target_swd_program_get_default_config(0);
		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tms_dir;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		program = &pico_target_swd_program;
		prog_config = pico_target_swd_program_get_default_config(0);
		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tck;
		break;

	default:
		assert(false);
		break;
	}

	assert(program != NULL);

	pio_add_program_at_offset(TARGET_SWD_PIO, program, 0);
	sm_config_set_set_pins(&prog_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&prog_config, sideset_pins_base);

	tap_pio_common_disable_input_sync(TARGET_SWD_PIO, target_pins->tms);
	sm_config_set_out_pins(&prog_config, target_pins->tms, 1);
	sm_config_set_in_pins(&prog_config, target_pins->tms);
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

static uint32_t swdptap_seq_in(size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_DRIVE) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_in_turnaround_out_to_in);
		tms_dir = SWDIO_STATUS_FLOAT;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_in);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);

	while (pio_sm_get_rx_fifo_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) < 1)
		;
	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) >> (32U - clock_cycles));

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
	pio_sm_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

	return value;
}

static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_DRIVE) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_in_turnaround_out_to_in);
		tms_dir = SWDIO_STATUS_FLOAT;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_in);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles);
	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->turnaround_in_to_out);

	while (pio_sm_get_rx_fifo_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) < 1)
		;

	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) >> (32U - clock_cycles));

	bool bit = false;
	if (clock_cycles == 32) {
		while (pio_sm_get_rx_fifo_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ < 1))
			;
		bit = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) != 0);
	} else {
		bit = ((value & (1u << clock_cycles)) != 0);
	}

	tms_dir = SWDIO_STATUS_DRIVE;
	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
	pio_sm_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

	const bool parity = calculate_odd_parity(value);
	*ret = value;
	return parity == bit;
}

static void swdptap_seq_out(const uint32_t tms_states, const size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out_turnaround_in_to_out);
		tms_dir = SWDIO_STATUS_DRIVE;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);
	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}

static void swdptap_seq_out_parity(const uint32_t tms_states, const size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	const bool parity = calculate_odd_parity(tms_states);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out_turnaround_in_to_out);
		tms_dir = SWDIO_STATUS_DRIVE;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles);

	if (clock_cycles <= 31) {
		uint32_t value = tms_states;
		value |= (parity ? (1u << clock_cycles) : 0);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, value);
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, parity ? (1 << 0) : 0);
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}

uint8_t rp2040_pio_adiv5_swd_raw_access_req(const uint8_t request)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, false);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, true);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, p_program_pos->adiv5_req_turnaround_in_to_out);
		tms_dir = SWDIO_STATUS_DRIVE;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, p_program_pos->adiv5_req);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, request);
	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);

	tms_dir = SWDIO_STATUS_FLOAT;
	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5) >> 29) & 0x7);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, true);

	return ack;
}

bool rp2040_pio_adiv5_swd_raw_access_data(const uint32_t data_out, uint32_t *data_in, const uint8_t rnw)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	uint32_t parity = false;

	if (rnw) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_in);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 32);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out_turnaround_in_to_out);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, TARGET_SWD_IDLE_CYCLES - 1);

		while (pio_sm_get_rx_fifo_level(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) < 2)
			;
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 0);

		*data_in = pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
		parity = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ) != 0) == calculate_odd_parity(*data_in);

		tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
		pio_sm_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
	} else {
		parity = calculate_odd_parity(data_out);

		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out_turnaround_in_to_out);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, 32 + TARGET_SWD_IDLE_CYCLES);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, data_out);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, (parity ? (1 << 0) : 0));

		tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
	}

	tms_dir = SWDIO_STATUS_DRIVE;

	return parity;
}

uint8_t rp2040_pio_adiv5_swd_write_no_check(const uint8_t request, const uint32_t data)
{
	const uint8_t ack = rp2040_pio_adiv5_swd_raw_access_req(request);
	rp2040_pio_adiv5_swd_raw_access_data(data, NULL, 0);

	return ack;
}

uint8_t rp2040_pio_adiv5_swd_read_no_check(const uint8_t request, uint32_t *data)
{
	const uint8_t ack = rp2040_pio_adiv5_swd_raw_access_req(request);
	rp2040_pio_adiv5_swd_raw_access_data(0, data, 1);

	return ack;
}

void swdptap_seq_out_buffer(const uint32_t *tms_states, const size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out_turnaround_in_to_out);
		tms_dir = SWDIO_STATUS_DRIVE;
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, p_program_pos->seq_out);
	}

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clock_cycles - 1);

	uint32_t data_count = clock_cycles / 32;
	if ((clock_cycles % 32) != 0) {
		data_count++;
	}

	for (size_t i = 0; i < data_count; i++) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, tms_states[i]);
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);
}