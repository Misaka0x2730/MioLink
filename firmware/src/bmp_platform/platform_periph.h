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

#ifndef MIOLINK_PLATFORM_PERIPH_H
#define MIOLINK_PLATFORM_PERIPH_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define PIN_NOT_CONNECTED (0xFF) /**< GPIO number sentinel for unpopulated pins */

#define TARGET_SERIAL_UART_MAIN    (uart1) /**< Primary target UART instance. */
#define TARGET_SERIAL_UART_TDI_TDO (uart0) /**< Alternate UART on TDI/TDO when muxed. */

#define TRACESWO_UART (uart0) /**< UART used for asynchronous SWO capture. */

#define TAP_PIO_SWD  (pio0) /**< PIO block instance for SWD operations. */
#define TAP_PIO_JTAG (pio0) /**< PIO block instance for JTAG operations. */

#endif /* MIOLINK_PLATFORM_PERIPH_H */
