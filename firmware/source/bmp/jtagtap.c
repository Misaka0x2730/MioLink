/*
 * This file is part of the Black Magic Debug project.
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

#include "general.h"
#include "platform.h"
#include "tap_pio_common.h"
#include "jtagtap.h"
#include "target_jtag.pio.h"

static void jtagtap_reset(void);
static void jtagtap_tms_seq(uint32_t tms_states, size_t ticks);
static void jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static void jtagtap_tdi_seq(bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static bool jtagtap_next(bool tms, bool tdi);
static void jtagtap_cycle(bool tms, bool tdi, size_t clock_cycles);

#define JTAG_PROGRAM_START_POS            (target_jtag_wrap_target)
#define JTAG_TDI_SEQ_POS                  (3)
#define JTAG_TDI_TDO_SEQ_POS              (9)
#define JTAG_TDI_TDO_SEQ_POS              (9)
#define JTAG_TDI_TDO_SEQ_FINAL_TMS_1_POS  (15)
#define JTAG_TDI_TDO_SEQ_FINAL_TMS_0_POS  (17)
#define JTAG_TMS_SEQ_POS                  (21)
#define JTAG_NEXT_CYCLE_INITIAL_SET_1_POS (24)
#define JTAG_NEXT_CYCLE_INITIAL_SET_0_POS (26)

#define JTAG_SM_NEXT_CYCLE_WRAP  (target_jtag_wrap)
#define JTAG_SM_TMS_SEQ_WRAP     (JTAG_NEXT_CYCLE_INITIAL_SET_1_POS - 1)
#define JTAG_SM_TDI_TDO_SEQ_WRAP (JTAG_TMS_SEQ_POS - 1)
#define JTAG_SM_TDI_SEQ_WRAP     (JTAG_TMS_SEQ_POS - 1)

jtag_proc_s jtag_proc;
extern uint32_t target_interface_frequency;

void jtagtap_init(void)
{
	tap_pio_common_disable_all_machines(TARGET_SWD_PIO);
	tap_pio_common_disable_all_machines(TARGET_JTAG_PIO);

	const platform_target_pins_t *target_pins = platform_get_target_pins();

	if (target_pins->tms_dir != PIN_NOT_CONNECTED) {
		gpio_init(target_pins->tms_dir);
		gpio_set_dir(target_pins->tms_dir, GPIO_OUT);
		gpio_put(target_pins->tms_dir, true);
	}

	gpio_init(target_pins->tck);
	pio_gpio_init(TARGET_JTAG_PIO, target_pins->tck);

	gpio_init(target_pins->tdo);
	gpio_set_pulls(target_pins->tdo, true, false);
	pio_gpio_init(TARGET_JTAG_PIO, target_pins->tdo);

	gpio_init(target_pins->tdi);
	pio_gpio_init(TARGET_JTAG_PIO, target_pins->tdi);

	gpio_init(target_pins->tms);
	pio_gpio_init(TARGET_JTAG_PIO, target_pins->tms);

	pio_clear_instruction_memory(TARGET_JTAG_PIO);

	const uint32_t pindirs_value = (1 << target_pins->tck) | (1 << target_pins->tdi) | (1 << target_pins->tms);
	const uint32_t pindirs_mask =
		(1 << target_pins->tck) | (1 << target_pins->tdi) | (1 << target_pins->tdo) | (1 << target_pins->tms);
	const uint32_t pins_value = 0;
	const uint32_t pins_mask = (1 << target_pins->tck) | (1 << target_pins->tdi) | (1 << target_pins->tms);

	pio_sm_set_pindirs_with_mask(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, pindirs_value, pindirs_mask);
	pio_sm_set_pins_with_mask(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, pins_value, pins_mask);

	tap_pio_common_disable_input_sync(TARGET_JTAG_PIO, target_pins->tdo);

	pio_add_program_at_offset(TARGET_JTAG_PIO, &target_jtag_program, JTAG_PROGRAM_START_POS);

	pio_sm_config prog_config = target_jtag_program_get_default_config(0);

	/* JTAG next and cycle SM */
	sm_config_set_wrap(&prog_config, JTAG_PROGRAM_START_POS, JTAG_SM_NEXT_CYCLE_WRAP);
	sm_config_set_in_pins(&prog_config, target_pins->tdo);
	sm_config_set_out_pins(&prog_config, target_pins->tdi, 1);
	sm_config_set_sideset_pins(&prog_config, target_pins->tck);
	sm_config_set_set_pins(&prog_config, target_pins->tms, 1);
	sm_config_set_out_shift(&prog_config, true, true, 8);
	sm_config_set_in_shift(&prog_config, true, true, 8);

	pio_sm_init(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, 0, &prog_config);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, false);

	/* JTAG TMS sequence SM */
	sm_config_set_wrap(&prog_config, JTAG_PROGRAM_START_POS, JTAG_SM_TMS_SEQ_WRAP);
	sm_config_set_in_pins(&prog_config, target_pins->tdo);
	sm_config_set_out_pins(&prog_config, target_pins->tms, 1);
	sm_config_set_sideset_pins(&prog_config, target_pins->tck);
	sm_config_set_set_pins(&prog_config, target_pins->tdi, 1);
	sm_config_set_out_shift(&prog_config, true, true, 8);
	sm_config_set_in_shift(&prog_config, true, true, 8);

	pio_sm_init(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, 0, &prog_config);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, false);

	/* JTAG TDI/TDO sequence SM */
	sm_config_set_wrap(&prog_config, JTAG_PROGRAM_START_POS, JTAG_SM_TDI_TDO_SEQ_WRAP);
	sm_config_set_in_pins(&prog_config, target_pins->tdo);
	sm_config_set_out_pins(&prog_config, target_pins->tdi, 1);
	sm_config_set_sideset_pins(&prog_config, target_pins->tck);
	sm_config_set_set_pins(&prog_config, target_pins->tms, 1);
	sm_config_set_out_shift(&prog_config, true, true, 8);
	sm_config_set_in_shift(&prog_config, true, true, 8);

	pio_sm_init(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, 0, &prog_config);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, false);

	/* JTAG TDI sequence SM */
	sm_config_set_wrap(&prog_config, JTAG_PROGRAM_START_POS, JTAG_SM_TDI_SEQ_WRAP);
	sm_config_set_in_pins(&prog_config, target_pins->tdo);
	sm_config_set_out_pins(&prog_config, target_pins->tdi, 1);
	sm_config_set_sideset_pins(&prog_config, target_pins->tck);
	sm_config_set_set_pins(&prog_config, target_pins->tms, 1);
	sm_config_set_out_shift(&prog_config, true, true, 8);
	sm_config_set_in_shift(&prog_config, true, true, 8);

	pio_sm_init(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, 0, &prog_config);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, false);

	platform_max_frequency_set(target_interface_frequency);

	jtag_proc.jtagtap_reset = jtagtap_reset;
	jtag_proc.jtagtap_next = jtagtap_next;
	jtag_proc.jtagtap_tms_seq = jtagtap_tms_seq;
	jtag_proc.jtagtap_tdi_tdo_seq = jtagtap_tdi_tdo_seq;
	jtag_proc.jtagtap_tdi_seq = jtagtap_tdi_seq;
	jtag_proc.jtagtap_cycle = jtagtap_cycle;
	jtag_proc.tap_idle_cycles = 1;

	/* Ensure we're in JTAG mode */
	for (size_t i = 0; i <= 50U; ++i)
		jtagtap_next(true, false); /* 50 + 1 idle cycles for SWD reset */
	jtagtap_tms_seq(0xe73cU, 16U); /* SWD to JTAG sequence */
}

static void jtagtap_reset(void)
{
	jtagtap_soft_reset();
}

static bool jtagtap_next(const bool tms, const bool tdi)
{
	uint8_t pio_buffer[PIO_BUFFER_SIZE] = { 0 };
	uint8_t data_amount = 0;

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, true);

	pio_buffer[data_amount++] = 0;
	pio_buffer[data_amount++] = tms ? JTAG_NEXT_CYCLE_INITIAL_SET_1_POS : JTAG_NEXT_CYCLE_INITIAL_SET_0_POS;
	pio_buffer[data_amount++] = (tdi ? 1 : 0);

	tap_pio_common_dma_send_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, pio_buffer, data_amount);
	const bool result = (pio_sm_get_blocking(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE) != 0);

	tap_pio_common_wait_for_tx_stall(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE);

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, false);

	return result;
}

static void jtagtap_tms_seq(const uint32_t tms_states, const size_t ticks)
{
	uint8_t pio_buffer[PIO_BUFFER_SIZE] = { 0 };
	uint8_t data_amount = 0;

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, true);

	pio_buffer[data_amount++] = (ticks - 1);
	pio_buffer[data_amount++] = JTAG_TMS_SEQ_POS;

	for (uint8_t i = 0, ticks_sent = 0; ((i < sizeof(tms_states)) && (ticks_sent < ticks)); i++, ticks_sent += 8)
	{
		pio_buffer[data_amount++] = ((tms_states >> (8 * i)) & 0xFF);
	}

	tap_pio_common_dma_send_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, pio_buffer, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ);

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, false);
}

static void jtagtap_tdi_tdo_seq(
	uint8_t *const data_out, const bool final_tms, const uint8_t *const data_in, size_t clock_cycles)
{
	uint8_t pio_buffer[PIO_BUFFER_SIZE] = { 0 };
	uint8_t data_amount = 0;

	if (clock_cycles == 0) {
		return;
	}

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, true);

	if (clock_cycles == 1) {
		pio_buffer[data_amount++] = 0;

		if (final_tms) {
			pio_buffer[data_amount++] = JTAG_TDI_TDO_SEQ_FINAL_TMS_1_POS;
		} else {
			pio_buffer[data_amount++] = JTAG_TDI_TDO_SEQ_FINAL_TMS_0_POS;
		}
		pio_buffer[data_amount++] = data_in[0];

		tap_pio_common_dma_send_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, pio_buffer, data_amount);

		data_out[0] = (pio_sm_get_blocking(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ) != 0) ? 1 : 0;
	} else {
		pio_buffer[data_amount++] = (clock_cycles - 2);
		pio_buffer[data_amount++] = JTAG_TDI_TDO_SEQ_POS;
		pio_buffer[data_amount++] = (final_tms ? 1 : 0);

		size_t data_bytes = clock_cycles / 8;
		if (clock_cycles % 8) {
			data_bytes++;
		}

		assert((data_amount + data_bytes) <= PIO_BUFFER_SIZE);
		memcpy(&(pio_buffer[data_amount]), data_in, data_bytes);

		data_amount += data_bytes;

		size_t data_out_cnt = tap_pio_common_dma_send_recv_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, pio_buffer, data_out, data_amount, data_bytes);

		if ((clock_cycles % 8) != 0) {
			data_out[data_out_cnt - 1] >>= (8 - (clock_cycles % 8));
		}
	}

	tap_pio_common_wait_for_tx_stall(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ);

	pio_sm_clear_fifos(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ);

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, false);
}

static void jtagtap_tdi_seq(const bool final_tms, const uint8_t *const data_in, const size_t clock_cycles)
{
	uint8_t pio_buffer[PIO_BUFFER_SIZE] = { 0 };
	uint8_t data_amount = 0;

	if (clock_cycles == 0) {
		return;
	}

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, true);

	if (clock_cycles == 1) {
		pio_buffer[data_amount++] = 0;

		if (final_tms) {
			pio_buffer[data_amount++] = JTAG_TDI_TDO_SEQ_FINAL_TMS_1_POS;
		} else {
			pio_buffer[data_amount++] = JTAG_TDI_TDO_SEQ_FINAL_TMS_0_POS;
		}
		pio_buffer[data_amount++] = data_in[0];

	} else {
		pio_buffer[data_amount++] = (clock_cycles - 2);
		pio_buffer[data_amount++] = JTAG_TDI_SEQ_POS;
		pio_buffer[data_amount++] = (final_tms ? 1 : 0);

		size_t data_bytes = (clock_cycles) / 8;
		if (clock_cycles % 8) {
			data_bytes++;
		}

		assert((data_amount + data_bytes) <= PIO_BUFFER_SIZE);
		memcpy(&(pio_buffer[data_amount]), data_in, data_bytes);

		data_amount += data_bytes;
	}

	tap_pio_common_dma_send_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, pio_buffer, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ);

	pio_sm_clear_fifos(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, false);
}

static void jtagtap_cycle(const bool tms, const bool tdi, const size_t clock_cycles)
{
	uint8_t pio_buffer[PIO_BUFFER_SIZE] = { 0 };
	uint8_t data_amount = 0;

	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, true);

	if (clock_cycles == 0) {
		return;
	}

	pio_buffer[data_amount++] = (clock_cycles - 1);

	if (tms) {
		pio_buffer[data_amount++] = JTAG_NEXT_CYCLE_INITIAL_SET_1_POS;
	} else {
		pio_buffer[data_amount++] = JTAG_NEXT_CYCLE_INITIAL_SET_0_POS;
	}

	pio_buffer[data_amount++] = (tdi ? 1 : 0);

	tap_pio_common_dma_send_recv_uint8(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, pio_buffer, NULL, data_amount, data_amount);

	tap_pio_common_wait_for_tx_stall(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE);

	pio_sm_clear_fifos(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE);
	pio_sm_set_enabled(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, false);
}