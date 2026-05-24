/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
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

#ifndef MIOLINK_USB_H
#define MIOLINK_USB_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

/**
 * \brief Maximum number of UTF-16 characters carried by a single USB string descriptor.
 *
 * The descriptor's \c bLength field is one byte and includes the 2-byte
 * (\c bLength + \c bDescriptorType) header. Each UTF-16 character takes 2 bytes, so the largest
 * descriptor that still fits is \c (255 - 2) / 2 = 126 characters. Buffers that back string
 * descriptors (the runtime UTF-16 scratch buffer, the ASCII \c board_ident source) should be
 * sized against this constant so that nothing produced by \c tud_descriptor_string_cb is
 * silently truncated.
 */
#define USB_STRING_DESCRIPTOR_MAX_CHARS (126U)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief USB device configuration state reported by \ref usb_get_config.
 */
typedef enum usb_config_state {
    USB_CONFIG_STATE_UNCONFIGURED = 0, /**< Device is not configured by the host. */
    USB_CONFIG_STATE_CONFIGURED = 1,   /**< Device is in its single active configuration. */
} usb_config_state_e;

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Initialise TinyUSB and related USB state for Black Magic.
 */
void blackmagic_usb_init(void);

/**
 * \brief Return the current USB device configuration state.
 *
 * The return type is \c uint16_t to match the upstream Black Magic prototype
 * (see \c platforms/common/usb.h), but the value is always one of
 * \ref usb_config_state_e and callers should compare against the enum constants.
 *
 * \return One of \ref usb_config_state_e values.
 */
uint16_t usb_get_config(void);

/**
 * \brief Whether USB configuration or interface alt-setting changed since last clear.
 *
 * \return \c true if the host updated configuration and the stack should react.
 */
bool usb_config_is_updated(void);

/**
 * \brief Clear the “configuration updated” latch after handling.
 */
void usb_config_clear_updated(void);

#endif /* MIOLINK_USB_H */
