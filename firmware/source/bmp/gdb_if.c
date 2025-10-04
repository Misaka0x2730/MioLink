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

/*
 * This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented. This implementation for STM32
 * uses the USB CDC-ACM device bulk endpoints to implement the channel.
 */

#include "general.h"
#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"
#include "platform_timing.h"

#include "tusb.h"

#include "usb_cdc.h"
#include "usb.h"

#include "gdb_if.h"

static uint32_t gdb_to_usb_count;
static char gdb_to_usb_buf[1024U];

bool gdb_serial_get_dtr(void)
{
	return (tud_cdc_n_get_line_state(USB_CDC_GDB) & 0x01) != 0;
}

void gdb_if_putchar(const char character, const bool flush)
{
	gdb_to_usb_buf[gdb_to_usb_count++] = character;
	if (flush || gdb_to_usb_count == sizeof(gdb_to_usb_buf)) {
		/* Refuse to send if USB isn't configured, and
		 * don't bother if nobody's listening */
		if (usb_get_config() != 1 || !gdb_serial_get_dtr()) {
			gdb_to_usb_count = 0;
			return;
		}

		uint32_t pos_in = 0;
		while (gdb_to_usb_count > 0) {
			const uint32_t avail = tud_cdc_n_write_available(USB_CDC_GDB);
			if (avail == 0) {
				vTaskDelay(pdMS_TO_TICKS(1));
			} else if (avail >= gdb_to_usb_count) {
				tud_cdc_n_write(USB_CDC_GDB, gdb_to_usb_buf + pos_in, gdb_to_usb_count);
				gdb_to_usb_count = 0;
			} else {
				uint32_t written = tud_cdc_n_write(USB_CDC_GDB, gdb_to_usb_buf + pos_in, avail);
				pos_in += written;
				gdb_to_usb_count -= avail;
			}
		}

		if (flush) {
			tud_cdc_n_write_flush(USB_CDC_GDB);
		}
	}
}

static uint8_t usb_to_gdb_buf[1024U] = {0};
static uint32_t usb_to_gdb_count = 0;
static uint32_t usb_to_gdb_buf_pos = 0;

char gdb_if_getchar(void)
{
	uint32_t notification_value = 0;

	do {
		if (!gdb_serial_get_dtr()) {
			return '\x04';
		}

		if (usb_get_config() != 1) {
			continue;
		}

		if (usb_to_gdb_buf_pos != usb_to_gdb_count) {
			return (char)usb_to_gdb_buf[usb_to_gdb_buf_pos++];
		}

		usb_to_gdb_buf_pos = 0;
		usb_to_gdb_count = 0;

		if (tud_cdc_n_available(USB_CDC_GDB) > 0) {
			usb_to_gdb_count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb_buf, sizeof(usb_to_gdb_buf));
			continue;
		}

		if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, pdMS_TO_TICKS(portMAX_DELAY)) != pdFALSE) {
			if (notification_value & USB_CDC_NOTIF_USB_RX_AVAILABLE) {
				usb_to_gdb_count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb_buf, sizeof(usb_to_gdb_buf));
				continue;
			}
		}
	} while (1);
}

char gdb_if_getchar_to(const uint32_t timeout)
{
	uint32_t notification_value = 0;

	if (!gdb_serial_get_dtr()) {
		return '\x04';
	}

	if (usb_get_config() != 1) {
		return -1;
	}

	if (usb_to_gdb_buf_pos != usb_to_gdb_count) {
		return (char)usb_to_gdb_buf[usb_to_gdb_buf_pos++];
	}

	usb_to_gdb_buf_pos = 0;
	usb_to_gdb_count = 0;

	if (tud_cdc_n_available(USB_CDC_GDB) > 0) {
		usb_to_gdb_count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb_buf, sizeof(usb_to_gdb_buf));
		if (usb_to_gdb_count > 0) {
			return (char)usb_to_gdb_buf[usb_to_gdb_buf_pos++];
		}
	}

	platform_timeout_s receive_timeout;
	platform_timeout_set(&receive_timeout, timeout);

	while (!platform_timeout_is_expired(&receive_timeout)) {
		const uint32_t timeout_left = platform_timeout_time_left(&receive_timeout);

		if ((timeout_left == 0) ||
			(xTaskNotifyWait(0, UINT32_MAX, &notification_value, pdMS_TO_TICKS(timeout_left)) != pdFALSE)) {
			if (notification_value & USB_CDC_NOTIF_USB_RX_AVAILABLE) {
				usb_to_gdb_count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb_buf, sizeof(usb_to_gdb_buf));
				if (usb_to_gdb_count > 0) {
					return (char)usb_to_gdb_buf[usb_to_gdb_buf_pos++];
				}
			}

			if (timeout_left == 0) {
				break;
			}
		}
	}

	return -1;
}