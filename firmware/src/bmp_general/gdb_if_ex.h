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

#ifndef MIOLINK_GDB_IF_EX_H
#define MIOLINK_GDB_IF_EX_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "usb_cdc.h"

#include <stdbool.h>

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief GDB CDC: query whether the host has asserted DTR.
 *
 * Local extension to the upstream Black Magic \c gdb_if.h interface; implemented
 * alongside \c gdb_if_putchar / \c gdb_if_getchar in \c gdb_if.c.
 *
 * \return \ref USB_CDC_DTR_ASSERTED when DTR is active (port "open" from the host perspective),
 *         \ref USB_CDC_DTR_DEASSERTED otherwise.
 */
bool gdb_serial_get_dtr(void);

#endif /* MIOLINK_GDB_IF_EX_H */
