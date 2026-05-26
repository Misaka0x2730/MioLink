/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#ifndef MIOLINK_USB_CDC_H
#define MIOLINK_USB_CDC_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define USB_CDC_NOTIF_USB_RX_AVAILABLE    (0x01) /**< Host→device USB OUT data available (task notify bit). */
#define USB_CDC_NOTIF_LINE_STATE_UPDATE   (0x02) /**< DTR/RTS line state change from host. */
#define USB_CDC_NOTIF_LINE_CODING_UPDATE  (0x04) /**< Baud/format (\c line_coding) update from host. */
#define USB_CDC_NOTIF_SERIAL_RX_AVAILABLE (0x08) /**< Target UART RX data ready (DMA or IRQ path). */
#define USB_CDC_NOTIF_SERIAL_RX_TIMEOUT   (0x10) /**< Target UART RX idle timeout. */
#define USB_CDC_NOTIF_SERIAL_TX_COMPLETE  (0x20) /**< Target UART TX DMA finished. */
#define USB_CDC_NOTIF_DUMMY               (0x80) /**< Placeholder / internal notify bit. */

/**
 * \brief \ref gdb_serial_get_dtr / \ref target_serial_get_dtr return value when the host has asserted DTR
 *        (port "open").
 */
#define USB_CDC_DTR_ASSERTED (true)

/**
 * \brief \ref gdb_serial_get_dtr / \ref target_serial_get_dtr return value when the host has not asserted DTR
 *        (port "closed").
 */
#define USB_CDC_DTR_DEASSERTED (false)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief Logical CDC interface index for composite USB (GDB, target UART, …).
 */
typedef enum {
    USB_CDC_GDB = 0,       /**< CDC 0: GDB remote serial protocol channel. */
    USB_CDC_TARGET_SERIAL, /**< CDC 1: target UART bridge channel. */
    USB_CDC_NUM,           /**< Number of CDC interfaces; must match TinyUSB \c CFG_TUD_CDC. */
} usb_cdc_t;

_Static_assert(USB_CDC_NUM == CFG_TUD_CDC);

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Register a FreeRTOS task as the listener for CDC events on a given interface.
 *
 * Registration must be visible by the time the USB task delivers a TinyUSB callback for the
 * interface. In practice, call this either before \ref blackmagic_usb_init for the interface,
 * or with the FreeRTOS scheduler suspended across init and registration so the USB task cannot
 * run in between. Only one listener per interface is tracked; a second call overwrites the
 * previous binding. The mask filters which \c USB_CDC_NOTIF_* bits reach the task — bits
 * absent from the mask are silently dropped at the callback site.
 *
 * \param[in] cdc         Target CDC interface.
 * \param[in] task        FreeRTOS task to notify; \c NULL disables the slot.
 * \param[in] accept_mask Bitmask of \c USB_CDC_NOTIF_* bits the task is willing to receive.
 */
void usb_cdc_register_listener(usb_cdc_t cdc, TaskHandle_t task, uint32_t accept_mask);

#endif /* MIOLINK_USB_CDC_H */
