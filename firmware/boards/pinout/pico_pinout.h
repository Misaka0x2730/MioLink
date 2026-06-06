/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2026 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#ifndef MIOLINK_BOARDS_PINOUT_PICO_H
#define MIOLINK_BOARDS_PINOUT_PICO_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define PICO_LED_ACT_PIN (25) /**< Activity LED GPIO (on-board user LED). */

#define PICO_TARGET_TCK_PIN (10) /**< JTAG/SWD clock pin. */
#define PICO_TARGET_TDO_PIN (13) /**< JTAG TDO / SWO input pin. */
#define PICO_TARGET_TDI_PIN (12) /**< JTAG TDI pin. */
#define PICO_TARGET_TMS_PIN (11) /**< JTAG TMS / SWD SWDIO pin. */

#define PICO_TARGET_UART_TX_PIN (8) /**< Target UART TX pin. */
#define PICO_TARGET_UART_RX_PIN (9) /**< Target UART RX pin. */

#define PICO_TARGET_NRST_PIN (7) /**< Target reset (nRST) GPIO. */

/* SWD PIO configuration for Raspberry Pi Pico used as a probe (matches \c pio_swd_pico.pio). */
#define PICO_SWD_PIO_SET_PIN_COUNT     (1) /**< Pin driven by \c set: TMS only. */
#define PICO_SWD_PIO_SIDESET_PIN_COUNT (1) /**< Sideset pin count (excluding the optional bit). */
#define PICO_SWD_PIO_SIDESET_PIN_BASE  (PICO_TARGET_TCK_PIN) /**< First sideset pin: TCK. */

#endif /* MIOLINK_BOARDS_PINOUT_PICO_H */
