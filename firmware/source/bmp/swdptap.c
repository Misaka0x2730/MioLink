/*
 * This file is part of the Black Magic Debug project.
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

#include "general.h"
#include "platform.h"
#include "swd.h"
#include "maths_utils.h"

#include "hardware/pio.h"
#include "miolink_rev_a_swd.pio.h"
#include "miolink_rev_b_swd.pio.h"
#include "miolink_pico_swd.pio.h"
#include "pico_swd.pio.h"

#include "tap_pio_common.h"

static uint32_t swdptap_seq_in(size_t clock_cycles);
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles);
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles);
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles);

typedef struct {
	uint8_t swd_in_out_to_in;
	uint8_t swd_in;
	uint8_t swd_in_parity_out_to_in;
	uint8_t swd_in_parity;
	uint8_t swd_out_in_to_out;
	uint8_t swd_out;
	uint8_t swd_adiv5_req_in_to_out;
	uint8_t swd_adiv5_req;
} swd_program_pos_t;

const swd_program_pos_t miolink_rev_a_program_pos = {
	.swd_in_out_to_in = miolink_rev_a_swd_in_program.origin,
	.swd_in = miolink_rev_a_swd_in_program.origin + miolink_rev_a_swd_in_wrap_target,
	.swd_in_parity_out_to_in = miolink_rev_a_swd_in_parity_program.origin,
	.swd_in_parity = miolink_rev_a_swd_in_parity_program.origin + miolink_rev_a_swd_in_parity_wrap_target,
	.swd_out_in_to_out = miolink_rev_a_swd_out_program.origin,
	.swd_out = miolink_rev_a_swd_out_program.origin + miolink_rev_a_swd_out_wrap_target,
	.swd_adiv5_req_in_to_out = miolink_rev_a_swd_adiv5_req_program.origin,
	.swd_adiv5_req = miolink_rev_a_swd_adiv5_req_program.origin + miolink_rev_a_swd_adiv5_req_wrap_target};

const swd_program_pos_t miolink_rev_b_program_pos = {
	.swd_in_out_to_in = miolink_rev_b_swd_in_program.origin,
	.swd_in = miolink_rev_b_swd_in_program.origin + miolink_rev_b_swd_in_wrap_target,
	.swd_in_parity_out_to_in = miolink_rev_b_swd_in_parity_program.origin,
	.swd_in_parity = miolink_rev_b_swd_in_parity_program.origin + miolink_rev_b_swd_in_parity_wrap_target,
	.swd_out_in_to_out = miolink_rev_b_swd_out_program.origin,
	.swd_out = miolink_rev_b_swd_out_program.origin + miolink_rev_b_swd_out_wrap_target,
	.swd_adiv5_req_in_to_out = miolink_rev_b_swd_adiv5_req_program.origin,
	.swd_adiv5_req = miolink_rev_b_swd_adiv5_req_program.origin + miolink_rev_b_swd_adiv5_req_wrap_target};

const swd_program_pos_t miolink_pico_program_pos = {
	.swd_in_out_to_in = miolink_pico_swd_in_program.origin,
	.swd_in = miolink_pico_swd_in_program.origin + miolink_pico_swd_in_wrap_target,
	.swd_in_parity_out_to_in = miolink_pico_swd_in_parity_program.origin,
	.swd_in_parity = miolink_pico_swd_in_parity_program.origin + miolink_pico_swd_in_parity_wrap_target,
	.swd_out_in_to_out = miolink_pico_swd_out_program.origin,
	.swd_out = miolink_pico_swd_out_program.origin + miolink_pico_swd_out_wrap_target,
	.swd_adiv5_req_in_to_out = miolink_pico_swd_adiv5_req_program.origin,
	.swd_adiv5_req = miolink_pico_swd_adiv5_req_program.origin + miolink_pico_swd_adiv5_req_wrap_target};

const swd_program_pos_t pico_program_pos = {
	.swd_in_out_to_in = pico_swd_in_program.origin,
	.swd_in = pico_swd_in_program.origin + pico_swd_in_wrap_target,
	.swd_in_parity_out_to_in = pico_swd_in_parity_program.origin,
	.swd_in_parity = pico_swd_in_parity_program.origin + pico_swd_in_parity_wrap_target,
	.swd_out_in_to_out = pico_swd_out_program.origin,
	.swd_out = pico_swd_out_program.origin + pico_swd_out_wrap_target,
	.swd_adiv5_req_in_to_out = pico_swd_adiv5_req_program.origin,
	.swd_adiv5_req = pico_swd_adiv5_req_program.origin + pico_swd_adiv5_req_wrap_target};

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
			p_program_pos = &miolink_rev_a_program_pos;
		} else {
			p_program_pos = &miolink_rev_b_program_pos;
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
	tap_pio_common_disable_all_machines(TARGET_SWD_PIO);
	tap_pio_common_disable_all_machines(TARGET_JTAG_PIO);

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

	pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, tms_dir_mask | (1 << target_pins->tck),
		tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));
	pio_sm_set_pins_with_mask(
		TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, 0, tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));

	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	pio_sm_config swd_in_config = { 0 };
	pio_sm_config swd_in_parity_config = { 0 };
	pio_sm_config swd_out_config = { 0 };
	pio_sm_config swd_adiv5_req_config = { 0 };
	uint8_t set_pins_base = PIN_NOT_CONNECTED;
	uint8_t set_pins_count = 1;
	uint8_t sideset_pins_base = PIN_NOT_CONNECTED;

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_a_swd_in_program, miolink_rev_a_swd_in_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_a_swd_in_parity_program, miolink_rev_a_swd_in_parity_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_a_swd_out_program, miolink_rev_a_swd_out_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_a_swd_adiv5_req_program, miolink_rev_a_swd_adiv5_req_program.origin);

			swd_in_config = miolink_rev_a_swd_in_program_get_default_config(miolink_rev_a_swd_in_program.origin);
			swd_in_parity_config = miolink_rev_a_swd_in_parity_program_get_default_config(miolink_rev_a_swd_in_parity_program.origin);
			swd_out_config = miolink_rev_a_swd_out_program_get_default_config(miolink_rev_a_swd_out_program.origin);
			swd_adiv5_req_config = miolink_rev_a_swd_adiv5_req_program_get_default_config(miolink_rev_a_swd_adiv5_req_program.origin);

			set_pins_base = target_pins->tms;
			set_pins_count = 2;
			sideset_pins_base = target_pins->tck;
		} else {
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_b_swd_in_program, miolink_rev_b_swd_in_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_b_swd_in_parity_program, miolink_rev_b_swd_in_parity_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_b_swd_out_program, miolink_rev_b_swd_out_program.origin);
			pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_rev_b_swd_adiv5_req_program, miolink_rev_b_swd_adiv5_req_program.origin);

			swd_in_config = miolink_rev_b_swd_in_program_get_default_config(miolink_rev_b_swd_in_program.origin);
			swd_in_parity_config = miolink_rev_b_swd_in_parity_program_get_default_config(miolink_rev_b_swd_in_parity_program.origin);
			swd_out_config = miolink_rev_b_swd_out_program_get_default_config(miolink_rev_b_swd_out_program.origin);
			swd_adiv5_req_config = miolink_rev_b_swd_adiv5_req_program_get_default_config(miolink_rev_b_swd_adiv5_req_program.origin);

			set_pins_base = target_pins->tms;
			set_pins_count = 1;
			sideset_pins_base = target_pins->tck;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_pico_swd_in_program, miolink_pico_swd_in_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_pico_swd_in_parity_program, miolink_pico_swd_in_parity_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_pico_swd_out_program, miolink_pico_swd_out_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &miolink_pico_swd_adiv5_req_program, miolink_pico_swd_adiv5_req_program.origin);

		swd_in_config = miolink_pico_swd_in_program_get_default_config(miolink_pico_swd_in_program.origin);
		swd_in_parity_config = miolink_pico_swd_in_parity_program_get_default_config(miolink_pico_swd_in_parity_program.origin);
		swd_out_config = miolink_pico_swd_out_program_get_default_config(miolink_pico_swd_out_program.origin);
		swd_adiv5_req_config = miolink_pico_swd_adiv5_req_program_get_default_config(miolink_pico_swd_adiv5_req_program.origin);

		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tms_dir;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		pio_add_program_at_offset(TARGET_SWD_PIO, &pico_swd_in_program, pico_swd_in_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &pico_swd_in_parity_program, pico_swd_in_parity_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &pico_swd_out_program, pico_swd_out_program.origin);
		pio_add_program_at_offset(TARGET_SWD_PIO, &pico_swd_adiv5_req_program, pico_swd_adiv5_req_program.origin);

		swd_in_config = pico_swd_in_program_get_default_config(pico_swd_in_program.origin);
		swd_in_parity_config = pico_swd_in_parity_program_get_default_config(pico_swd_in_parity_program.origin);
		swd_out_config = pico_swd_out_program_get_default_config(pico_swd_out_program.origin);
		swd_adiv5_req_config = pico_swd_adiv5_req_program_get_default_config(pico_swd_adiv5_req_program.origin);

		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tck;

		gpio_set_pulls(target_pins->tms, true, false);
		break;

	default:
		assert(false);
		break;
	}

	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	tap_pio_common_disable_input_sync(TARGET_SWD_PIO, target_pins->tms);

	sm_config_set_set_pins(&swd_in_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&swd_in_config, sideset_pins_base);
	sm_config_set_in_pins(&swd_in_config, target_pins->tms);
	sm_config_set_out_pins(&swd_in_config, target_pins->tms, 1);
	sm_config_set_out_shift(&swd_in_config, true, true, 32);
	sm_config_set_in_shift(&swd_in_config, true, true, 32);

	pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, p_program_pos->swd_in, &swd_in_config);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, false);

	sm_config_set_set_pins(&swd_in_parity_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&swd_in_parity_config, sideset_pins_base);
	sm_config_set_in_pins(&swd_in_parity_config, target_pins->tms);
	sm_config_set_out_pins(&swd_in_parity_config, target_pins->tms, 1);
	sm_config_set_out_shift(&swd_in_parity_config, true, true, 32);
	sm_config_set_in_shift(&swd_in_parity_config, true, true, 32);

	pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, p_program_pos->swd_in_parity, &swd_in_parity_config);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, false);

	sm_config_set_set_pins(&swd_out_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&swd_out_config, sideset_pins_base);
	sm_config_set_in_pins(&swd_out_config, target_pins->tms);
	sm_config_set_out_pins(&swd_out_config, target_pins->tms, 1);
	sm_config_set_out_shift(&swd_out_config, true, true, 32);
	sm_config_set_in_shift(&swd_out_config, true, true, 32);

	pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, p_program_pos->swd_out, &swd_out_config);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, false);

	sm_config_set_set_pins(&swd_adiv5_req_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&swd_adiv5_req_config, sideset_pins_base);
	sm_config_set_in_pins(&swd_adiv5_req_config, target_pins->tms);
	sm_config_set_out_pins(&swd_adiv5_req_config, target_pins->tms, 1);
	sm_config_set_out_shift(&swd_adiv5_req_config, true, true, 8);
	sm_config_set_in_shift(&swd_adiv5_req_config, true, true, 3);

	pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, p_program_pos->swd_adiv5_req, &swd_adiv5_req_config);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);

	platform_max_frequency_set(target_interface_frequency);

	tms_dir = SWDIO_STATUS_FLOAT;

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
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, pio_encode_jmp(p_program_pos->swd_in_out_to_in));
		tms_dir = SWDIO_STATUS_FLOAT;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, clock_cycles - 1);
	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN) >> (32U - clock_cycles));

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN);
	pio_sm_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN, false);

	return value;
}

static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_DRIVE) {
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, pio_encode_jmp(p_program_pos->swd_in_parity_out_to_in));
		tms_dir = SWDIO_STATUS_FLOAT;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, clock_cycles);
	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY) >> (32U - clock_cycles));

	bool bit = false;
	if (clock_cycles == 32) {
		bit = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY) != 0);
	} else {
		bit = ((value & (1UL << clock_cycles)) != 0);
	}

	tms_dir = SWDIO_STATUS_DRIVE;
	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY);
	pio_sm_clear_fifos(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, false);

	const bool parity = calculate_odd_parity(value);
	*ret = value;
	return parity == bit;
}

static void swdptap_seq_out(const uint32_t tms_states, const size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, pio_encode_jmp(p_program_pos->swd_out_in_to_out));
		tms_dir = SWDIO_STATUS_DRIVE;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, clock_cycles - 1);
	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, tms_states);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, false);
}

static void swdptap_seq_out_parity(const uint32_t tms_states, const size_t clock_cycles)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	const bool parity = calculate_odd_parity(tms_states);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, pio_encode_jmp(p_program_pos->swd_out_in_to_out));
		tms_dir = SWDIO_STATUS_DRIVE;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, clock_cycles);

	if (clock_cycles <= 31) {
		uint32_t value = tms_states;
		value |= (parity ? (1UL << clock_cycles) : 0);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, value);
	} else {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, tms_states);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, parity ? (1 << 0) : 0);
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT);
}

uint8_t rp2040_pio_adiv5_swd_raw_access_req(const uint8_t request)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	if (tms_dir == SWDIO_STATUS_FLOAT) {
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, pio_encode_jmp(p_program_pos->swd_adiv5_req_in_to_out));
		tms_dir = SWDIO_STATUS_DRIVE;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, request);

	tms_dir = SWDIO_STATUS_FLOAT;
	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5) >> 29) & 0x7);
	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, false);

	return ack;
}

bool rp2040_pio_adiv5_swd_raw_access_data(const uint32_t data_out, uint32_t *data_in, const uint8_t rnw)
{
	const swd_program_pos_t *p_program_pos = swdtap_get_program_pos();
	assert(p_program_pos != NULL);

	bool parity = false;

	if (rnw) {
		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, true);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, 32);

		*data_in = pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY);
		parity =
			(pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY) != 0) == (calculate_odd_parity(*data_in) != 0);
		tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY);
		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_IN_PARITY, false);

		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, true);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, TARGET_SWD_IDLE_CYCLES - 1);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, 0);
		tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT);
		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, false);
	} else {
		parity = (calculate_odd_parity(data_out) != 0);

		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, true);
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, pio_encode_jmp(p_program_pos->swd_out_in_to_out));
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, 32 + TARGET_SWD_IDLE_CYCLES);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, data_out);
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, (parity ? (1 << 0) : 0));

		tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT);
		pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, false);
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
		pio_sm_exec(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, pio_encode_jmp(p_program_pos->swd_out_in_to_out));
		tms_dir = SWDIO_STATUS_DRIVE;
	}

	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, true);

	pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, clock_cycles - 1);

	uint32_t data_count = clock_cycles / 32;
	if ((clock_cycles % 32) != 0) {
		data_count++;
	}

	for (size_t i = 0; i < data_count; i++) {
		pio_sm_put_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, tms_states[i]);
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ_OUT, false);
}