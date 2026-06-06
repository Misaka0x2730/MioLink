/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
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

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "platform.h"
#include "platform_timing.h"

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

#include "usb_cdc.h"
#include "usb.h"
#include "gdb_if.h"
#include "gdb_if_ex.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#if !defined(CONFIG_GDB_IF_BUFFER_SIZE)
#error "CONFIG_GDB_IF_BUFFER_SIZE must be defined by the build system"
#endif

#define GDB_IF_TX_WAIT_MS (1U) /**< Back-off delay (ms) when the USB TX FIFO has no space available. */

/**
 * \brief Back-off delay (ms) while DTR stays asserted but the USB device is not configured.
 *
 * Guards \ref gdb_if_getchar against a busy-loop in the pathological window where the host
 * has driven DTR=1 without (or before) a valid USB configuration; a tick-sized wait yields
 * the CPU while we wait for either \c USB_CDC_NOTIF_LINE_STATE_UPDATE or enumeration to
 * complete.
 */
#define GDB_IF_UNCONFIGURED_WAIT_MS (5U)

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Staging buffer for one direction of the GDB CDC channel.
 *
 * The \c pos cursor is only meaningful for the inbound (host-to-GDB) direction, where it tracks how many bytes
 * have already been consumed from \c buf. The outbound (GDB-to-host) direction leaves \c pos unused.
 */
typedef struct {
    uint8_t buf[CONFIG_GDB_IF_BUFFER_SIZE]; /**< Byte storage for queued bytes. */
    uint32_t count;                         /**< Number of valid bytes currently held in \c buf. */
    uint32_t pos;                           /**< Read cursor into \c buf (unused for outbound direction). */
} gdb_if_buffer_s;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

static gdb_if_buffer_s gdb_to_usb = {0}; /**< Outbound buffer: GDB protocol bytes pending transmission to host. */
static gdb_if_buffer_s usb_to_gdb = {0}; /**< Inbound buffer: bytes received from host awaiting GDB consumption. */

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

bool gdb_serial_get_dtr(void)
{
    return (tud_cdc_n_get_line_state(USB_CDC_GDB) & 0x01) != 0;
}

/**
 * \brief Enqueue a single GDB protocol byte; flush to USB on \a flush or when the buffer is full.
 *
 * \param[in] character Byte to enqueue.
 * \param[in] flush     Force flush to USB after appending.
 */
void gdb_if_putchar(const char character, const bool flush)
{
    gdb_to_usb.buf[gdb_to_usb.count++] = (uint8_t)character;
    if ((flush) || (gdb_to_usb.count == sizeof(gdb_to_usb.buf))) {
        /* Refuse to send if USB isn't configured, and
         * don't bother if nobody's listening */
        if ((usb_get_config() != USB_CONFIG_STATE_CONFIGURED) ||
            (gdb_serial_get_dtr() == USB_CDC_DTR_DEASSERTED)) {
            gdb_to_usb.count = 0;
            return;
        }

        uint32_t buf_pos = 0;
        while (gdb_to_usb.count > 0) {
            /* Host may close the CDC mid-transmission; re-check on each iteration so the task
             * does not spin forever waiting for FIFO space that will never be drained. */
            if ((usb_get_config() != USB_CONFIG_STATE_CONFIGURED) ||
                (gdb_serial_get_dtr() == USB_CDC_DTR_DEASSERTED)) {
                gdb_to_usb.count = 0;
                return;
            }

            const uint32_t avail = tud_cdc_n_write_available(USB_CDC_GDB);
            const uint32_t bytes_to_write = MIN(avail, gdb_to_usb.count);

            if (bytes_to_write == 0) {
                /* Nothing can be written right now, wait a bit */
                vTaskDelay(pdMS_TO_TICKS(GDB_IF_TX_WAIT_MS));
            } else {
                const uint32_t written_bytes = tud_cdc_n_write(USB_CDC_GDB, gdb_to_usb.buf + buf_pos, bytes_to_write);
                buf_pos += written_bytes;
                gdb_to_usb.count -= written_bytes;
            }
        }

        if (flush) {
            tud_cdc_n_write_flush(USB_CDC_GDB);
        }
    }
}

/**
 * \brief Blocking read of one GDB protocol byte from the host.
 *
 * \return Next received byte, or \c '\x04' if DTR is dropped (treated as end-of-stream by GDB).
 */
char gdb_if_getchar(void)
{
    uint32_t notification_value = 0;

    do {
        if (gdb_serial_get_dtr() != USB_CDC_DTR_ASSERTED) {
            return '\x04';
        }

        if (usb_get_config() != USB_CONFIG_STATE_CONFIGURED) {
            vTaskDelay(pdMS_TO_TICKS(GDB_IF_UNCONFIGURED_WAIT_MS));
            continue;
        }

        if (usb_to_gdb.pos != usb_to_gdb.count) {
            return (char)usb_to_gdb.buf[usb_to_gdb.pos++];
        }

        usb_to_gdb.pos = 0;
        usb_to_gdb.count = 0;

        if (tud_cdc_n_available(USB_CDC_GDB) > 0) {
            usb_to_gdb.count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb.buf, sizeof(usb_to_gdb.buf));
            continue;
        }

        if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, portMAX_DELAY) != pdFALSE) {
            if (notification_value & USB_CDC_NOTIF_USB_RX_AVAILABLE) {
                usb_to_gdb.count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb.buf, sizeof(usb_to_gdb.buf));
            }
        }
    } while (1);
}

/**
 * \brief Read one GDB protocol byte with a millisecond timeout.
 *
 * \param[in] timeout Maximum time to wait, in milliseconds. Pass \c 0 to poll.
 * \return Next received byte, \c '\x04' if DTR is dropped, or \c -1 on timeout / no configured device.
 */
char gdb_if_getchar_to(const uint32_t timeout)
{
    uint32_t notification_value = 0;

    if (gdb_serial_get_dtr() != USB_CDC_DTR_ASSERTED) {
        return '\x04';
    }

    if (usb_get_config() != USB_CONFIG_STATE_CONFIGURED) {
        return -1;
    }

    if (usb_to_gdb.pos != usb_to_gdb.count) {
        return (char)usb_to_gdb.buf[usb_to_gdb.pos++];
    }

    usb_to_gdb.pos = 0;
    usb_to_gdb.count = 0;

    if (tud_cdc_n_available(USB_CDC_GDB) > 0) {
        usb_to_gdb.count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb.buf, sizeof(usb_to_gdb.buf));
        if (usb_to_gdb.count > 0) {
            return (char)usb_to_gdb.buf[usb_to_gdb.pos++];
        }
    }

    platform_timeout_s receive_timeout = {0};
    platform_timeout_set(&receive_timeout, timeout);

    while (!platform_timeout_is_expired(&receive_timeout)) {
        const uint32_t timeout_left = platform_timeout_time_left(&receive_timeout);

        if (timeout_left == 0) {
            break;
        }

        if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, pdMS_TO_TICKS(timeout_left)) != pdFALSE) {
            if (notification_value & USB_CDC_NOTIF_USB_RX_AVAILABLE) {
                usb_to_gdb.count = tud_cdc_n_read(USB_CDC_GDB, usb_to_gdb.buf, sizeof(usb_to_gdb.buf));
                if (usb_to_gdb.count > 0) {
                    return (char)usb_to_gdb.buf[usb_to_gdb.pos++];
                }
            }
        }
    }

    return -1;
}
