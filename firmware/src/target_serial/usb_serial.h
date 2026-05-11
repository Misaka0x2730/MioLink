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

#ifndef MIOLINK_USB_SERIAL_H
#define MIOLINK_USB_SERIAL_H

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hardware/uart.h"

/**********************************************************************************************************************
 * Global Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Target-serial CDC: host DTR asserted.
 *
 * \return \c true if DTR is active (port “open” from host perspective).
 */
bool usb_serial_get_dtr(void);

/**
 * \brief Refresh activity/error LEDs from UART / USB serial state.
 */
void usb_serial_update_led(void);

/**
 * \brief Bytes waiting in the target UART RX software path.
 *
 * \return Number of bytes available to \c usb_serial_read.
 */
uint16_t usb_serial_get_available(void);

/**
 * \brief Read from the target UART bridge into \a data.
 *
 * \param data        Destination buffer.
 * \param buffer_size Capacity of \a data in bytes.
 * \return Bytes copied; may be less than \a buffer_size.
 */
uint32_t usb_serial_read(uint8_t *data, uint32_t buffer_size);

/**
 * \brief Enqueue payload to the target-serial CDC IN endpoint.
 *
 * \param data              Bytes to send; may be consumed or copied depending on path.
 * \param len               Length of \a data.
 * \param flush             Force immediate start / short packet if applicable.
 * \param allow_drop_buffer Allow dropping when USB TX is stalled (back-pressure policy).
 * \return \c false if the frame could not be accepted.
 */
bool usb_serial_send_to_usb(uint8_t *data, size_t len, bool flush, bool allow_drop_buffer);

/**
 * \brief Route target serial to UART on TDI/TDO pins instead of the main UART.
 *
 * \param new_state \c true to use TDI/TDO UART; \c false for main UART.
 */
void usb_serial_use_uart_on_tdi_tdo(const bool new_state);

/**
 * \brief Whether the TDI/TDO UART is currently selected for target serial.
 */
bool usb_serial_uart_on_tdi_tdo_is_used(void);

/**
 * \brief Mirror of \c usb_get_config() for modules that only include this header.
 *
 * \return Active USB configuration number.
 */
uint16_t usb_get_config(void);

/**
 * \brief GDB CDC: host DTR asserted.
 */
bool gdb_serial_get_dtr(void);

/**
 * \brief Start FreeRTOS task(s) and DMA for USB↔UART bridging.
 */
void usb_serial_init(void);

#endif /* MIOLINK_USB_SERIAL_H */
