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

#define SWDP_ACK_OK   (0x01U)

static uint32_t swdptap_seq_in(size_t clock_cycles);
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles);
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles);
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles);

typedef struct {
	const struct pio_program *swd_start_prog;
	const struct pio_program *swd_seq_out_turnaround;
	const struct pio_program *swd_seq_out;
	const struct pio_program *swd_seq_in_turnaround;
	const struct pio_program *swd_seq_in;
	const struct pio_program *swd_adiv5_check_ack;
	const struct pio_program *swd_turnaround_float_to_drive;
} swd_board_program_t;

const swd_board_program_t miolink_rev_a_programs = {
	.swd_start_prog = &miolink_rev_a_swd_start_program,
	.swd_seq_out_turnaround = &miolink_rev_a_swd_seq_out_turnaround_program,
	.swd_seq_out = &miolink_rev_a_swd_seq_out_program,
	.swd_seq_in_turnaround = &miolink_rev_a_swd_seq_in_turnaround_program,
	.swd_seq_in = &miolink_rev_a_swd_seq_in_program,
	.swd_adiv5_check_ack = &miolink_rev_a_swd_adiv5_check_ack_program,
	.swd_turnaround_float_to_drive = &miolink_rev_a_swd_turnaround_float_to_drive_program,
};

const swd_board_program_t miolink_rev_b_programs = {
	.swd_start_prog = &miolink_rev_b_swd_start_program,
	.swd_seq_out_turnaround = &miolink_rev_b_swd_seq_out_turnaround_program,
	.swd_seq_out = &miolink_rev_b_swd_seq_out_program,
	.swd_seq_in_turnaround = &miolink_rev_b_swd_seq_in_turnaround_program,
	.swd_seq_in = &miolink_rev_b_swd_seq_in_program,
	.swd_adiv5_check_ack = &miolink_rev_b_swd_adiv5_check_ack_program,
	.swd_turnaround_float_to_drive = &miolink_rev_b_swd_turnaround_float_to_drive_program,
};

const swd_board_program_t miolink_pico_programs = {
	.swd_start_prog = &miolink_pico_swd_start_program,
	.swd_seq_out_turnaround = &miolink_pico_swd_seq_out_turnaround_program,
	.swd_seq_out = &miolink_pico_swd_seq_out_program,
	.swd_seq_in_turnaround = &miolink_pico_swd_seq_in_turnaround_program,
	.swd_seq_in = &miolink_pico_swd_seq_in_program,
	.swd_adiv5_check_ack = &miolink_pico_swd_adiv5_check_ack_program,
	.swd_turnaround_float_to_drive = &miolink_pico_swd_turnaround_float_to_drive_program,
};

const swd_board_program_t pico_programs = {
	.swd_start_prog = &pico_swd_start_program,
	.swd_seq_out_turnaround = &pico_swd_seq_out_turnaround_program,
	.swd_seq_out = &pico_swd_seq_out_program,
	.swd_seq_in_turnaround = &pico_swd_seq_in_turnaround_program,
	.swd_seq_in = &pico_swd_seq_in_program,
	.swd_adiv5_check_ack = &pico_swd_adiv5_check_ack_program,
	.swd_turnaround_float_to_drive = &pico_swd_turnaround_float_to_drive_program,
};

typedef enum swdio_status_e {
	SWDIO_STATUS_FLOAT = 0,
	SWDIO_STATUS_DRIVE
} swdio_status_t;

swd_proc_s swd_proc;

static swdio_status_t tms_dir = SWDIO_STATUS_FLOAT;

extern uint32_t target_interface_frequency;

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

	pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, tms_dir_mask | (1 << target_pins->tck),
		tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));
	pio_sm_set_pins_with_mask(
		TARGET_SWD_PIO, TARGET_SWD_PIO_SM, 0, tms_dir_mask | (1 << target_pins->tck) | (1 << target_pins->tms));

	pio_sm_config swd_program_config = pio_get_default_sm_config();
	uint8_t set_pins_base = PIN_NOT_CONNECTED;
	uint8_t set_pins_count = 1;
	uint8_t sideset_pins_base = PIN_NOT_CONNECTED;

	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	const swd_board_program_t *p_board_programs = swdtap_get_board_programs();
	assert(p_board_programs != NULL);

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			sm_config_set_sideset(&swd_program_config, 2, true, false);

			set_pins_base = target_pins->tms;
			set_pins_count = 2;
			sideset_pins_base = target_pins->tck;
		} else {
			sm_config_set_sideset(&swd_program_config, 3, true, false);

			set_pins_base = target_pins->tms;
			set_pins_count = 1;
			sideset_pins_base = target_pins->tck;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		sm_config_set_sideset(&swd_program_config, 3, true, false);

		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tms_dir;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		sm_config_set_sideset(&swd_program_config, 2, true, false);

		set_pins_base = target_pins->tms;
		set_pins_count = 1;
		sideset_pins_base = target_pins->tck;

		gpio_set_pulls(target_pins->tms, true, false);
		break;

	default:
		assert(false);
		break;
	}

	tap_pio_common_disable_input_sync(TARGET_SWD_PIO, target_pins->tms);

	pio_clear_instruction_memory(TARGET_SWD_PIO);

	pio_add_program_at_offset(
		TARGET_SWD_PIO, p_board_programs->swd_start_prog, p_board_programs->swd_start_prog->origin);
	pio_add_program_at_offset(
		TARGET_SWD_PIO, p_board_programs->swd_seq_out_turnaround, p_board_programs->swd_seq_out_turnaround->origin);
	pio_add_program_at_offset(TARGET_SWD_PIO, p_board_programs->swd_seq_out, p_board_programs->swd_seq_out->origin);
	pio_add_program_at_offset(
		TARGET_SWD_PIO, p_board_programs->swd_seq_in_turnaround, p_board_programs->swd_seq_in_turnaround->origin);
	pio_add_program_at_offset(TARGET_SWD_PIO, p_board_programs->swd_seq_in, p_board_programs->swd_seq_in->origin);
	pio_add_program_at_offset(
		TARGET_SWD_PIO, p_board_programs->swd_adiv5_check_ack, p_board_programs->swd_adiv5_check_ack->origin);
	pio_add_program_at_offset(TARGET_SWD_PIO, p_board_programs->swd_turnaround_float_to_drive,
		p_board_programs->swd_turnaround_float_to_drive->origin);

	sm_config_set_set_pins(&swd_program_config, set_pins_base, set_pins_count);
	sm_config_set_sideset_pins(&swd_program_config, sideset_pins_base);
	sm_config_set_in_pins(&swd_program_config, target_pins->tms);
	sm_config_set_out_pins(&swd_program_config, target_pins->tms, 1);
	sm_config_set_out_shift(&swd_program_config, true, true, 32);
	sm_config_set_in_shift(&swd_program_config, true, true, 32);
	sm_config_set_wrap(&swd_program_config, p_board_programs->swd_start_prog->origin,
		p_board_programs->swd_turnaround_float_to_drive->origin +
			p_board_programs->swd_turnaround_float_to_drive->length - 1);

	pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, p_board_programs->swd_start_prog->origin, &swd_program_config);
	pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, true);

	platform_max_frequency_set(target_interface_frequency);

	tms_dir = SWDIO_STATUS_FLOAT;

	swd_proc.seq_in = swdptap_seq_in;
	swd_proc.seq_in_parity = swdptap_seq_in_parity;
	swd_proc.seq_out = swdptap_seq_out;
	swd_proc.seq_out_parity = swdptap_seq_out_parity;
}

static uint8_t swdtap_prepare_raw_pio_sequence(
	uint32_t *buffer, const uint32_t clock_cycles, const uint32_t data, const bool in, const bool parity)
{
	assert(buffer != NULL);
	assert(clock_cycles <= 32);

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

			if (clock_cycles <= 31) {
				uint32_t data_value = data;
				data_value |= (parity_value ? (1UL << clock_cycles) : 0);
				buffer[pos++] = data_value;
			} else {
				buffer[pos++] = data;
				buffer[pos++] = parity_value ? (1 << 0) : 0;
			}
		}
	} else if (!in) {
		buffer[pos++] = data;
	}

	return pos;
}

static uint32_t swdptap_seq_in(const size_t clock_cycles)
{
	assert(clock_cycles <= 32);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = swdtap_prepare_raw_pio_sequence(pio_buffer, clock_cycles, 0, true, false);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> (32U - clock_cycles));

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	return value;
}

static bool swdptap_seq_in_parity(uint32_t *ret, const size_t clock_cycles)
{
	assert(ret != NULL);
	assert(clock_cycles <= 32);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = swdtap_prepare_raw_pio_sequence(pio_buffer, clock_cycles, 0, true, true);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	const uint32_t value = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> (32U - clock_cycles));

	bool parity_read = false;
	if (clock_cycles == 32) {
		parity_read = (pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) != 0);
	} else {
		parity_read = ((value & (1UL << clock_cycles)) != 0);
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	const bool parity = (calculate_odd_parity(value) != 0);
	*ret = value;
	return parity == parity_read;
}

static void swdptap_seq_out(const uint32_t tms_states, const size_t clock_cycles)
{
	assert(clock_cycles <= 32);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = swdtap_prepare_raw_pio_sequence(pio_buffer, clock_cycles, tms_states, false, false);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);
}

static void swdptap_seq_out_parity(const uint32_t tms_states, const size_t clock_cycles)
{
	assert(clock_cycles <= 32);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = swdtap_prepare_raw_pio_sequence(pio_buffer, clock_cycles, tms_states, false, true);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);
}

void swdptap_seq_out_buffer(const uint32_t *tms_states, const size_t clock_cycles)
{
	assert(tms_states != NULL);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	uint8_t data_amount = 0;

	uint32_t data_count = clock_cycles / 32;
	if ((clock_cycles % 32) != 0) {
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

	assert((data_amount + data_count) <= PIO_BUFFER_SIZE);
	memcpy(&(pio_buffer[data_amount]), tms_states, data_count * sizeof(pio_buffer[0]));

	data_amount += data_count;

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);
}

static uint8_t rp2040_pio_swd_adiv5_prepare_pio_sequence(
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

	buffer[pos++] = 8 - 1;
	buffer[pos++] = request;
	buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in_turnaround->origin);
	buffer[pos++] = 3 - 1;

	if (check_ack) {
		buffer[pos++] = (uint32_t)(p_board_programs->swd_adiv5_check_ack->origin);
		buffer[pos++] = (SWDP_ACK_OK << 29);

		if (rnw) {
			buffer[pos++] = 5 - 1;
		} else {
			buffer[pos++] = 4 - 1;
		}
	}

	if (rnw) {
		buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_in->origin);
		buffer[pos++] = 32 + 1 - 1;
		buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
		buffer[pos++] = TARGET_SWD_IDLE_CYCLES - 1;
		buffer[pos++] = 0;
	} else {
		buffer[pos++] = (uint32_t)(p_board_programs->swd_seq_out_turnaround->origin);
		buffer[pos++] = 32 + 1 + TARGET_SWD_IDLE_CYCLES - 1;
		buffer[pos++] = data;
		buffer[pos++] = ((calculate_odd_parity(data) != 0) ? (1 << 0) : 0);
	}

	return pos;
}

uint8_t rp2040_pio_adiv5_swd_write_no_check(const uint8_t request, const uint32_t data)
{
	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = rp2040_pio_swd_adiv5_prepare_pio_sequence(pio_buffer, request, data, false, false);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> 29) & 0x7);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	return ack;
}

uint8_t rp2040_pio_adiv5_swd_read_no_check(const uint8_t request, uint32_t *data)
{
	assert(data != NULL);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = rp2040_pio_swd_adiv5_prepare_pio_sequence(pio_buffer, request, 0, true, false);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> 29) & 0x7);
	*data = pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);
	pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	return ack;
}

uint8_t rp2040_pio_adiv5_swd_write_check(const uint8_t request, const uint32_t data)
{
	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = rp2040_pio_swd_adiv5_prepare_pio_sequence(pio_buffer, request, data, false, true);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);

	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> 29) & 0x7);

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	return ack;
}

uint8_t rp2040_pio_adiv5_swd_read_check(const uint8_t request, uint32_t *data, bool *parity)
{
	assert(data != NULL);

	uint32_t pio_buffer[PIO_BUFFER_SIZE] = {0};
	const uint8_t data_amount = rp2040_pio_swd_adiv5_prepare_pio_sequence(pio_buffer, request, 0, true, true);

	tap_pio_common_dma_send_uint32(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, pio_buffer, data_amount);
	const uint8_t ack = (uint8_t)((pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) >> 29) & 0x7);

	if (ack == SWDP_ACK_OK) {
		*data = pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);
		const bool parity_value =
			(pio_sm_get_blocking(TARGET_SWD_PIO, TARGET_SWD_PIO_SM) != 0) == (calculate_odd_parity(*data) != 0);
		if (parity != NULL) {
			*parity = parity_value;
		}
	}

	tap_pio_common_wait_for_tx_stall(TARGET_SWD_PIO, TARGET_SWD_PIO_SM);

	return ack;
}