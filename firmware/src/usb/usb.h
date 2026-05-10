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

#ifndef MIOLINK_USB_H
#define MIOLINK_USB_H

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************************************
 * Global Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Initialise TinyUSB and related USB state for Black Magic.
 */
void blackmagic_usb_init(void);

/**
 * \brief Return the current USB configuration value (device configuration descriptor index).
 *
 * \return Active configuration number, or \c 0 if not configured.
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
