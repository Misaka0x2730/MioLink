/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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

#ifndef BMP_USB_SERIAL_H
#define BMP_USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#include "tusb.h"

typedef enum usb_serial_interface_e
{
	USB_SERIAL_GDB = 0,
	USB_SERIAL_TARGET,
	USB_SERIAL_NUM = CFG_TUD_CDC,
} usb_serial_interface_t;

#define USB_SERIAL_DATA_RX                (0x01)
#define USB_SERIAL_LINE_STATE_UPDATE      (0x02)
#define USB_SERIAL_LINE_CODING_UPDATE     (0x04)
#define USB_SERIAL_DATA_UART_RX_AVAILABLE (0x08)
#define USB_SERIAL_DATA_UART_RX_FLUSH     (0x10)
#define USB_SERIAL_DATA_UART_RX_TIMEOUT   (0x20)
#define USB_SERIAL_DATA_UART_TX_COMPLETE  (0x40)

extern bool use_uart_on_tdi_tdo;

uint16_t usb_get_config(void);
bool gdb_serial_get_dtr(void);
void usb_serial_init(void);

#endif /* BMP_USB_SERIAL_H */