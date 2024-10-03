/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2024 Dmitry Rezvanov <gareth@blacksphere.co.nz>
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

#ifndef MIOLINK_TAP_PIO_COMMON_H
#define MIOLINK_TAP_PIO_COMMON_H

#include "general.h"
#include "hardware/pio.h"

#define TARGET_SWD_PIO          (pio0)
#define TARGET_SWD_PIO_SM_SEQ   (0)
#define TARGET_SWD_PIO_SM_ADIV5 (1)

#define TARGET_JTAG_PIO                (pio0)
#define TARGET_JTAG_PIO_SM_NEXT_CYCLE  (0)
#define TARGET_JTAG_PIO_SM_TMS_SEQ     (1)
#define TARGET_JTAG_PIO_SM_TDI_TDO_SEQ (2)
#define TARGET_JTAG_PIO_SM_TDI_SEQ     (3)

static inline bool tap_pio_common_is_not_tx_stalled(PIO pio, uint32_t sm)
{
	pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
	return ((pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0);
}

static inline void tap_pio_common_wait_for_tx_stall(PIO pio, uint32_t sm)
{
	pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
	while ((pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0)
		;
}

static inline void tap_pio_common_disable_input_sync(PIO pio, uint32_t pin)
{
	pio->input_sync_bypass |= (1u << pin);
}

#endif //MIOLINK_TAP_PIO_COMMON_H