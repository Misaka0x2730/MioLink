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
#include "timing.h"
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

static void swdptap_turnaround(swdio_status_t dir) __attribute__((optimize(3)));
static uint32_t swdptap_seq_in(size_t clock_cycles) __attribute__((optimize(3)));
static bool swdptap_seq_in_parity(uint32_t *ret, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));
static void swdptap_seq_out_parity(uint32_t tms_states, size_t clock_cycles) __attribute__((optimize(3)));

/*
 * Overall strategy for timing consistency:
 *
 * - Each primitive ends with a falling clock edge
 * - Output is driven after the falling clock edge
 * - Input is read immediately before the rising clock edge
 * - Each primitive assumes it was immediately preceded by a falling clock edge
 *
 * This increases the chances of meeting setup and hold times when the target
 * connection is lower bandwidth (with adequately slower clocks configured).
 */

#define TARGET_NON_ISO_PIO (pio0)

#define SWD_START_PROG_POS            (0)
#define SWD_FLOAT_TO_DRIVE_POS        (1)
#define SWD_SEQ_OUT_POS               (3)
#define SWD_DRIVE_TO_FLOAT_POS        (8)
#define SWD_SEQ_IN_POS                (10)
#define SWD_DRIVE_TO_FLOAT_PARITY_POS (15)
#define SWD_SEQ_IN_PARITY_POS         (17)

static swdio_status_t olddir = SWDIO_STATUS_FLOAT;

extern uint32_t target_interface_frequency;

void swdptap_init(void)
{
    gpio_init(TARGET_TMS_DIR_PIN);
    gpio_init(TARGET_TCK_PIN);
    gpio_init(TARGET_TMS_PIN);

    pio_gpio_init(TARGET_NON_ISO_PIO, TARGET_TMS_DIR_PIN);
    pio_gpio_init(TARGET_NON_ISO_PIO, TARGET_TCK_PIN);
    pio_gpio_init(TARGET_NON_ISO_PIO, TARGET_TMS_PIN);

	pio_sm_set_enabled(TARGET_NON_ISO_PIO, 0, false);
	pio_clear_instruction_memory(TARGET_NON_ISO_PIO);

    pio_sm_set_pindirs_with_mask(TARGET_NON_ISO_PIO, 0, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN), (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));
    pio_sm_set_pins_with_mask(TARGET_NON_ISO_PIO, 0, 0, (1 << TARGET_TMS_DIR_PIN) | (1 << TARGET_TCK_PIN) | (1 << TARGET_TMS_PIN));

    pio_add_program_at_offset(TARGET_NON_ISO_PIO, &target_non_iso_swd_program, 0);
    pio_sm_config prog_config = target_non_iso_swd_program_get_default_config(0);
    sm_config_set_out_pins(&prog_config, TARGET_TMS_PIN, 1);
    sm_config_set_in_pins(&prog_config, TARGET_TMS_PIN);
    sm_config_set_sideset_pins(&prog_config, TARGET_TCK_PIN);
    sm_config_set_set_pins(&prog_config, TARGET_TMS_DIR_PIN, 2);
    sm_config_set_out_shift(&prog_config, true, true, 0);
    sm_config_set_in_shift(&prog_config, true, true, 0);

    TARGET_NON_ISO_PIO->input_sync_bypass |= (1u << TARGET_TMS_PIN);

    pio_sm_init(TARGET_NON_ISO_PIO, 0, 0, &prog_config);
    pio_sm_set_enabled(TARGET_NON_ISO_PIO, 0, true);

    platform_max_frequency_set(target_interface_frequency);

    swd_proc.seq_in = swdptap_seq_in;
	swd_proc.seq_in_parity = swdptap_seq_in_parity;
	swd_proc.seq_out = swdptap_seq_out;
	swd_proc.seq_out_parity = swdptap_seq_out_parity;
}

/*static void swdptap_turnaround(const swdio_status_t dir)
{
	if (dir == olddir)
		return;
	olddir = dir;

#ifdef DEBUG_SWD_BITS
	DEBUG_INFO("%s", dir ? "\n-> " : "\n<- ");
#endif

    pio_sm_clear_fifos(TARGET_NON_ISO_PIO, 0);
    TARGET_NON_ISO_PIO->irq |= (1 << 0);

	if (dir == SWDIO_STATUS_FLOAT)
    {
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, SWD_DRIVE_TO_FLOAT_POS);
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, SWD_START_PROG_POS);
	}
    else
    {
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, SWD_FLOAT_TO_DRIVE_FINAL_POS);
    }
    while((TARGET_NON_ISO_PIO->irq & (1 << 0)) == 0);
}*/

static uint32_t __not_in_flash_func(swdptap_seq_in)(size_t clock_cycles)
{
    hw_xor_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    //TARGET_NON_ISO_PIO->irq |= (1 << 0);

    if (olddir == SWDIO_STATUS_DRIVE)
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_DRIVE_TO_FLOAT_POS;
        olddir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_SEQ_IN_POS;
    }

    TARGET_NON_ISO_PIO->txf[0] = clock_cycles - 1;
    TARGET_NON_ISO_PIO->txf[0] = SWD_START_PROG_POS;
    //TARGET_NON_ISO_PIO->fdebug = (1ul << 24);
    //while((TARGET_NON_ISO_PIO->fdebug & (1 << 24)) == 0);
    //while((TARGET_NON_ISO_PIO->irq & (1 << 0)) == 0);

    while((TARGET_NON_ISO_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) != 0);
    uint32_t value = TARGET_NON_ISO_PIO->rxf[0];
    value >>= (32U - clock_cycles);

    return value;
}

static bool __not_in_flash_func(swdptap_seq_in_parity)(uint32_t *ret, size_t clock_cycles)
{
    hw_set_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_clear_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    //TARGET_NON_ISO_PIO->irq |= (1 << 0);

    if (olddir == SWDIO_STATUS_DRIVE)
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_DRIVE_TO_FLOAT_PARITY_POS;
        olddir = SWDIO_STATUS_FLOAT;
    }
    else
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_SEQ_IN_PARITY_POS;
    }

    TARGET_NON_ISO_PIO->txf[0] = clock_cycles;
    //while((TARGET_NON_ISO_PIO->irq & (1 << 0)) == 0);
    //TARGET_NON_ISO_PIO->fdebug = (1ul << 24);
    //while((TARGET_NON_ISO_PIO->fdebug & (1 << 24)) == 0);

    olddir = SWDIO_STATUS_DRIVE;

    while((TARGET_NON_ISO_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) != 0);
    uint32_t value = TARGET_NON_ISO_PIO->rxf[0];
    value >>= (32U - clock_cycles);

    bool bit = false;
    if (clock_cycles == 32)
    {
        while((TARGET_NON_ISO_PIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) != 0);
        bit = (TARGET_NON_ISO_PIO->rxf[0] != 0);
    }
    else
    {
        bit = ((value & (1ul << clock_cycles)) != 0);
    }

    const bool parity = calculate_odd_parity(value);
    *ret = value;
    return parity == bit;

	/*const uint32_t result = swdptap_seq_in(clock_cycles);
	const bool bit = swdptap_seq_in(1);

	swdptap_turnaround(SWDIO_STATUS_DRIVE);

	const bool parity = calculate_odd_parity(result);
	*ret = result;
	return parity == bit;*/
}

static void __not_in_flash_func(swdptap_seq_out)(const uint32_t tms_states, const size_t clock_cycles)
{
    hw_set_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);
    //TARGET_NON_ISO_PIO->irq |= (1 << 0);

    if (olddir == SWDIO_STATUS_FLOAT)
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_FLOAT_TO_DRIVE_POS;
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        TARGET_NON_ISO_PIO->txf[0] = SWD_SEQ_OUT_POS;
    }

    TARGET_NON_ISO_PIO->txf[0] = clock_cycles - 1;
    TARGET_NON_ISO_PIO->txf[0] = tms_states;
    if (clock_cycles == 32)
    {
        TARGET_NON_ISO_PIO->txf[0] = 0;
    }
    TARGET_NON_ISO_PIO->fdebug = (1ul << 24);
    //while((TARGET_NON_ISO_PIO->irq & (1 << 0)) == 0);
    while((TARGET_NON_ISO_PIO->fdebug & (1 << 24)) == 0);

    hw_clear_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);
}

static void __not_in_flash_func(swdptap_seq_out_parity)(const uint32_t tms_states, const size_t clock_cycles)
{
    const bool parity = calculate_odd_parity(tms_states);

    hw_set_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);
    //(&TARGET_NON_ISO_PIO->fdebug, (1ul << 24));

    TARGET_NON_ISO_PIO->irq |= (1 << 0);

    if (olddir == SWDIO_STATUS_FLOAT)
    {
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, SWD_FLOAT_TO_DRIVE_POS);
        olddir = SWDIO_STATUS_DRIVE;
    }
    else
    {
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, SWD_SEQ_OUT_POS);
    }

    pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, clock_cycles);

    if (clock_cycles <= 31)
    {
        uint32_t value = tms_states;
        value |= (parity ? (1ul << clock_cycles) : 0);
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, value);

        if (clock_cycles == 31)
        {
            pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, 0);
        }
    }
    else
    {
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, tms_states);
        pio_sm_put_blocking(TARGET_NON_ISO_PIO, 0, parity);
    }

    TARGET_NON_ISO_PIO->fdebug = (1ul << 24);
    //while((TARGET_NON_ISO_PIO->irq & (1 << 0)) == 0);
    while((TARGET_NON_ISO_PIO->fdebug & (1 << 24)) == 0);

    hw_clear_bits(&TARGET_NON_ISO_PIO->sm[0].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS);
}