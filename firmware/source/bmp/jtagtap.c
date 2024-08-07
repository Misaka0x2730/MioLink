/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022-2023 1BitSquared <info@1bitsquared.com>
 * Modified by Rachel Mant <git@dragonmux.network>
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

#include <stdio.h>

#include "general.h"
#include "platform.h"
#include "jtagtap.h"
#include "target_jtag.pio.h"

jtag_proc_s jtag_proc;

static void jtagtap_reset(void);
static void jtagtap_tms_seq(uint32_t tms_states, size_t ticks);
static void jtagtap_tdi_tdo_seq(uint8_t *data_out, bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static void jtagtap_tdi_seq(bool final_tms, const uint8_t *data_in, size_t clock_cycles);
static bool jtagtap_next(bool tms, bool tdi);
static void jtagtap_cycle(bool tms, bool tdi, size_t clock_cycles);

#define JTAG_START_PROGRAM_POS        (0)
#define JTAG_OUT_SEQ_SET_1_POS        (1)
#define JTAG_OUT_SEQ_SET_0_POS        (3)
#define JTAG_OUT_SEQ_POS              (4)
#define JTAG_TDI_TDO_SEQ_POS          (9)
#define JTAG_FINAL_TMS_1_POS          (16)
#define JTAG_FINAL_TMS_0_POS          (18)
#define JTAG_CYCLE_POS                (24)

extern uint32_t target_interface_frequency;

void jtagtap_init(void)
{
    gpio_init(TARGET_TCK_PIN);
    gpio_set_dir(TARGET_TCK_PIN, GPIO_OUT);

    gpio_init(TARGET_TDO_PIN);

    gpio_init(TARGET_TDI_PIN);
    gpio_set_dir(TARGET_TDI_PIN, GPIO_OUT);

    gpio_init(TARGET_TMS_PIN);
    gpio_set_dir(TARGET_TMS_PIN, GPIO_OUT);

    gpio_init(TARGET_TMS_DIR_PIN);
    gpio_set_dir(TARGET_TMS_DIR_PIN, GPIO_OUT);
    gpio_put(TARGET_TMS_DIR_PIN, true);

    gpio_set_pulls(TARGET_TDO_PIN, true, false);

    pio_gpio_init(TARGET_SWD_PIO, TARGET_TCK_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TDO_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TDI_PIN);
    pio_gpio_init(TARGET_SWD_PIO, TARGET_TMS_PIN);

    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, false);
    pio_clear_instruction_memory(TARGET_SWD_PIO);

    pio_sm_set_pindirs_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM,  (1 << TARGET_TCK_PIN) | (1 << TARGET_TDI_PIN) | (1 << TARGET_TMS_PIN), (1 << TARGET_TCK_PIN) | (1 << TARGET_TDI_PIN)  | (1 << TARGET_TDO_PIN) | (1 << TARGET_TMS_PIN));
    pio_sm_set_pins_with_mask(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, 0, (1 << TARGET_TCK_PIN) | (1 << TARGET_TDI_PIN) | (1 << TARGET_TMS_PIN));

    pio_add_program_at_offset(TARGET_SWD_PIO, &target_jtag_program, 0);
    pio_sm_config prog_config = target_jtag_program_get_default_config(0);
    sm_config_set_out_pins(&prog_config, TARGET_TDI_PIN, 1);
    sm_config_set_in_pins(&prog_config, TARGET_TDO_PIN);
    sm_config_set_sideset_pins(&prog_config, TARGET_TCK_PIN);
    sm_config_set_set_pins(&prog_config, TARGET_TMS_PIN, 1);
    sm_config_set_out_shift(&prog_config, true, true, 8);
    sm_config_set_in_shift(&prog_config, true, true, 8);

    TARGET_SWD_PIO->input_sync_bypass |= (1u << TARGET_TDO_PIN);

    pio_sm_init(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, 0, &prog_config);
    pio_sm_set_enabled(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, true);

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
#ifdef TRST_PORT
	if (platform_hwversion() == 0) {
		gpio_clear(TRST_PORT, TRST_PIN);
		for (volatile size_t i = 0; i < 10000U; i++)
			continue;
		gpio_set(TRST_PORT, TRST_PIN);
	}
#endif
	jtagtap_soft_reset();
}

static bool jtagtap_next(const bool tms, const bool tdi)
{
    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    if (tms)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_1_POS;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_0_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (tdi ? (1 << 0) : 0);

    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);

    const bool result = (TARGET_SWD_PIO->rxf[0] != 0);

    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    return result;
}

static void jtagtap_tms_seq(const uint32_t tms_states, const size_t ticks)
{
    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    pio_sm_set_set_pins(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, TARGET_TDI_PIN, 1);
    pio_sm_set_out_pins(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, TARGET_TMS_PIN, 1);

    hw_write_masked(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl,
                    ((uint32_t)(0) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
                    ((uint32_t)(0) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB),
                    PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_OUT_SEQ_SET_1_POS;
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (ticks - 1);
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = tms_states;

    /*if (ticks == 32)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;
    }*/

    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);

    hw_write_masked(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl,
                    ((uint32_t)(8) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
                    ((uint32_t)(8) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB),
                    PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS);

    hw_set_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    pio_sm_set_set_pins(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, TARGET_TMS_PIN, 1);
    pio_sm_set_out_pins(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, TARGET_TDI_PIN, 1);
}

static void jtagtap_tdi_tdo_seq(
	uint8_t *const data_out, const bool final_tms, const uint8_t *const data_in, size_t clock_cycles)
{
    if (clock_cycles == 0)
    {
        return;
    }

    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    if (clock_cycles == 1)
    {
        if (final_tms)
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_1_POS;
        }
        else
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_0_POS;
        }

        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = data_in[0];

        while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + TARGET_SWD_PIO_SM))) != 0);
        data_out[0] = (TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM]) ? (1 << 0) : 0;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_TDI_TDO_SEQ_POS;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles - 2;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (final_tms ? 1 : 0);

        size_t data_bytes = clock_cycles / 8;
        if (clock_cycles % 8)
        {
            data_bytes++;
        }

        size_t data_out_cnt = 0;
        for (size_t i = 0; i < data_bytes; i++)
        {
            while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = data_in[i];

            /*while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + TARGET_SWD_PIO_SM))) != 0);
            data_out[data_out_cnt++] = (uint8_t) ((TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> 24) & 0xFF);*/

            if ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + TARGET_SWD_PIO_SM))) == 0)
            {
                data_out[data_out_cnt++] = (uint8_t) ((TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> 24) & 0xFF);
            }
        }

        while (data_out_cnt < data_bytes)
        {
            while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + TARGET_SWD_PIO_SM))) != 0);
            data_out[data_out_cnt++] = (uint8_t) ((TARGET_SWD_PIO->rxf[TARGET_SWD_PIO_SM] >> 24) & 0xFF);
        }

        if ((clock_cycles % 8) == 0)
        {
            /*while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;*/
        }
        else
        {
            data_out[data_out_cnt - 1] >>= (8 - (clock_cycles % 8));
        }
    }

    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);

    hw_set_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

static void jtagtap_tdi_seq(const bool final_tms, const uint8_t *const data_in, const size_t clock_cycles)
{
    if (clock_cycles == 0)
    {
        return;
    }
    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    if (clock_cycles == 1)
    {
        if (final_tms)
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_1_POS;
        }
        else
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_0_POS;
        }

        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = data_in[0];
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_OUT_SEQ_SET_0_POS;
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = clock_cycles - 2;

        const size_t clock_cycles_no_final = (clock_cycles - 1);
        size_t data_bytes_no_final = clock_cycles_no_final / 8;
        if (clock_cycles_no_final % 8)
        {
            data_bytes_no_final++;
        }
        size_t final_tdi_byte = (clock_cycles / 8);
        if (clock_cycles % 8)
        {
            final_tdi_byte++;
        }
        bool final_tdi = ((data_in[final_tdi_byte - 1] & (1 << ((clock_cycles - 1) % 8))) != 0);
        for (size_t i = 0; i < data_bytes_no_final; i++)
        {
            while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = data_in[i];
        }

        if ((clock_cycles_no_final % 8) == 0)
        {
            /*while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = 0;*/
        }

        while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);

        if (final_tms)
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_1_POS;
        }
        else
        {
            TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_0_POS;
        }
        while ((TARGET_SWD_PIO->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + TARGET_SWD_PIO_SM))) != 0);
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (final_tdi ? 1 : 0);
    }

    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);

    hw_set_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

static void jtagtap_cycle(const bool tms, const bool tdi, const size_t clock_cycles)
{
    if (clock_cycles == 0)
    {
        return;
    }

    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    if (tms)
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_1_POS;
    }
    else
    {
        TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_FINAL_TMS_0_POS;
    }

    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (tdi ? (1 << 0) : 0);
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = JTAG_CYCLE_POS;
    TARGET_SWD_PIO->txf[TARGET_SWD_PIO_SM] = (clock_cycles - 1);

    TARGET_SWD_PIO->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM));
    while((TARGET_SWD_PIO->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + TARGET_SWD_PIO_SM))) == 0);

    hw_set_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}