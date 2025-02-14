/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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

#ifndef BMP_USB_SERIAL_H
#define BMP_USB_SERIAL_H

#include <stdint.h>
#include <stdbool.h>

#include "hardware/uart.h"
#include "tusb.h"

typedef enum usb_serial_interface_e {
	USB_SERIAL_GDB = 0,
	USB_SERIAL_TARGET,
	USB_SERIAL_NUM = CFG_TUD_CDC,
} usb_serial_interface_t;

#define USB_SERIAL_DATA_RX                (0x01)
#define USB_SERIAL_LINE_STATE_UPDATE      (0x02)
#define USB_SERIAL_LINE_CODING_UPDATE     (0x04)
#define USB_SERIAL_DATA_RX_AVAILABLE      (0x08)
#define USB_SERIAL_DATA_RX_FLUSH          (0x10)
#define USB_SERIAL_DATA_RX_TIMEOUT        (0x20)
#define USB_SERIAL_DATA_TX_COMPLETE       (0x40)

extern bool use_uart_on_tdi_tdo;

bool usb_serial_get_dtr(void);
void usb_serial_update_led(void);
uint16_t usb_serial_get_available(void);
uint32_t usb_serial_read(uint8_t *data, uint32_t buffer_size);
bool usb_serial_send_to_usb(uint8_t *data, size_t len, bool flush, bool allow_drop_buffer);

uint16_t usb_get_config(void);
bool gdb_serial_get_dtr(void);
void usb_serial_uart_release(uart_inst_t *uart_to_release);
void usb_serial_init(void);

#endif /* BMP_USB_SERIAL_H */