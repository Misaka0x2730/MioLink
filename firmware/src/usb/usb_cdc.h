/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Dmitry Rezvanov <git@dragonmux.network>
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
 * Includes
 **********************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "tusb.h"

/**********************************************************************************************************************
 * Global Types
 **********************************************************************************************************************/

/**
 * \brief Logical CDC interface index for composite USB (GDB, target UART, …).
 */
typedef enum {
	USB_CDC_GDB = 0,
	USB_CDC_TARGET_SERIAL,
	USB_CDC_NUM = CFG_TUD_CDC,
} usb_cdc_t;

/**********************************************************************************************************************
 * Global Definitions
 **********************************************************************************************************************/

#define USB_CDC_NOTIF_USB_RX_AVAILABLE    (0x01) /**< Host→device USB OUT data available (task notify bit). */
#define USB_CDC_NOTIF_LINE_STATE_UPDATE   (0x02) /**< DTR/RTS line state change from host. */
#define USB_CDC_NOTIF_LINE_CODING_UPDATE  (0x04) /**< Baud/format (\c line_coding) update from host. */
#define USB_CDC_NOTIF_SERIAL_RX_AVAILABLE (0x08) /**< Target UART RX data ready (DMA or IRQ path). */
#define USB_CDC_NOTIF_SERIAL_RX_TIMEOUT   (0x10) /**< Target UART RX idle timeout. */
#define USB_CDC_NOTIF_SERIAL_TX_COMPLETE  (0x20) /**< Target UART TX DMA finished. */
#define USB_CDC_NOTIF_DUMMY               (0x80) /**< Placeholder / internal notify bit. */

#endif /* MIOLINK_USB_CDC_H */
