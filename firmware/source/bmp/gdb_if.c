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

/*
 * This file implements a transparent channel over which the GDB Remote
 * Serial Debugging protocol is implemented. This implementation for STM32
 * uses the USB CDC-ACM device bulk endpoints to implement the channel.
 */

#include "general.h"
#include "platform.h"
#include "usb_serial.h"
#include "tusb.h"
#include "gdb_if.h"

static uint32_t count_out;
static uint32_t count_in;
static uint32_t out_ptr;
static char buffer_out[64];
static char buffer_in[64];

void __not_in_flash_func(gdb_if_putchar)(const char c, const int flush)
{
	buffer_in[count_in++] = c;
	if (flush || count_in == sizeof(buffer_in))
	{
		/* Refuse to send if USB isn't configured, and
		 * don't bother if nobody's listening */
		if (usb_get_config() != 1 || !gdb_serial_get_dtr())
		{
			count_in = 0;
			return;
		}

        while (tud_cdc_n_write_available(USB_SERIAL_GDB) < count_in);

        if (tud_cdc_n_write(USB_SERIAL_GDB, buffer_in, count_in) != count_in)
        {
            vTaskDelay(1);
        }

		if (flush && (count_in < sizeof(buffer_in)))
		{
			tud_cdc_n_write_flush(USB_SERIAL_GDB);
		}

		count_in = 0;
	}
}

char __not_in_flash_func(gdb_if_getchar)(void)
{
    uint32_t notificationValue = 0;

    do
    {
        if (!gdb_serial_get_dtr())
        {
            return '\x04';
        }

        if (usb_get_config() != 1)
        {
            continue;
        }

        if (tud_cdc_n_available(USB_SERIAL_GDB) > 0)
        {
            return (char)tud_cdc_n_read_char(USB_SERIAL_GDB);
        }

        if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(portMAX_DELAY)) != pdFALSE)
        {
            if (notificationValue & USB_SERIAL_DATA_RX)
            {
                return (char)tud_cdc_n_read_char(USB_SERIAL_GDB);
            }
        }
    } while (1);

	/*while (tud_cdc_n_available(USB_SERIAL_GDB) == 0)
	{
		if (!gdb_serial_get_dtr())
		{
			return '\x04';
		}

        while (usb_get_config() != 1)
            vTaskDelay(5);

        //vTaskDelay(1);

        taskYIELD();
	}

    return (char)tud_cdc_n_read_char(USB_SERIAL_GDB);*/
}

char __not_in_flash_func(gdb_if_getchar_to)(const uint32_t timeout)
{
    uint32_t notificationValue = 0;

    if (!gdb_serial_get_dtr())
    {
        return '\x04';
    }

    if (usb_get_config() != 1)
    {
        return -1;
    }

    if (tud_cdc_n_available(USB_SERIAL_GDB) > 0)
    {
        return (char)tud_cdc_n_read_char(USB_SERIAL_GDB);
    }

    if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(timeout)) != pdFALSE)
    {
        if (notificationValue & USB_SERIAL_DATA_RX)
        {
            return (char)tud_cdc_n_read_char(USB_SERIAL_GDB);
        }
    }

    return -1;

	/*platform_timeout_s receive_timeout;
	platform_timeout_set(&receive_timeout, timeout);

	while ((tud_cdc_n_available(USB_SERIAL_GDB) == 0) && !platform_timeout_is_expired(&receive_timeout))
	{
        if (!gdb_serial_get_dtr())
        {
            return '\x04';
        }

        while (usb_get_config() != 1)
            vTaskDelay(5);

        //vTaskDelay(1);
	}

    return (tud_cdc_n_available(USB_SERIAL_GDB) > 0) ? (char)tud_cdc_n_read_char(USB_SERIAL_GDB) : -1;*/
}