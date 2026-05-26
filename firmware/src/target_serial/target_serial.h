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

#ifndef MIOLINK_TARGET_SERIAL_H
#define MIOLINK_TARGET_SERIAL_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "hardware/uart.h"

#include "usb_cdc.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Target-serial CDC: host DTR asserted.
 *
 * \return \ref USB_CDC_DTR_ASSERTED when DTR is active (port "open" from the host perspective),
 *         \ref USB_CDC_DTR_DEASSERTED otherwise.
 */
bool target_serial_get_dtr(void);

/**
 * \brief Refresh activity/error LEDs from UART / USB serial state.
 */
void target_serial_update_led(void);

/**
 * \brief Bytes waiting in the target UART RX software path.
 *
 * \return Number of bytes available to \c target_serial_read.
 */
uint16_t target_serial_get_available(void);

/**
 * \brief Read from the target UART bridge into \a data.
 *
 * \param[out] data        Destination buffer.
 * \param[in]  buffer_size Capacity of \a data in bytes.
 * \return Bytes copied; may be less than \a buffer_size.
 */
uint32_t target_serial_read(uint8_t *data, uint32_t buffer_size);

/**
 * \brief Enqueue payload to the target-serial CDC IN endpoint.
 *
 * Calling with \a len == 0 is treated as an explicit flush request: the call is reported
 * as successful and the CDC IN endpoint is flushed regardless of the \a flush argument.
 *
 * \param[in] data              Bytes to send; may be consumed or copied depending on path.
 *                              Ignored when \a len is 0.
 * \param[in] len               Length of \a data. Pass 0 to flush only.
 * \param[in] flush             Force immediate start / short packet if applicable. Implicitly
 *                              forced to \c true when \a len is 0.
 * \param[in] allow_drop_buffer Allow dropping when USB TX is stalled (back-pressure policy).
 * \return \c false if the frame could not be accepted.
 */
bool target_serial_send_to_usb(uint8_t *data, size_t len, bool flush, bool allow_drop_buffer);

/**
 * \brief Route target serial to UART on TDI/TDO pins instead of the main UART.
 *
 * \param[in] new_state \c true to use TDI/TDO UART; \c false for main UART.
 */
void target_serial_use_uart_on_tdi_tdo(const bool new_state);

/**
 * \brief Whether the TDI/TDO UART is currently selected for target serial.
 *
 * \return \c true if the TDI/TDO-pin UART is the active sink/source; \c false for the main UART.
 */
bool target_serial_uart_on_tdi_tdo_is_used(void);

/**
 * \brief Surrender the TDI/TDO UART so that the JTAG TAP can repurpose the pins.
 *
 * Asserts a sticky lockout that suppresses the user-visible UART-on-TDI/TDO selection until
 * \ref target_serial_tap_release_tdi_tdo clears it.  If the bridge currently owns the TDI/TDO
 * UART, the call synchronously detaches the UART peripheral and returns the GPIOs to SIO so
 * that JTAG can reassign them to PIO immediately on return.  Intended to be invoked from
 * \c jtagtap_init before it touches TDI/TDO GPIOs.
 */
void target_serial_tap_acquire_tdi_tdo(void);

/**
 * \brief Release the TDI/TDO UART lockout previously asserted by \ref target_serial_tap_acquire_tdi_tdo.
 *
 * After this call, if the user has enabled UART-on-TDI/TDO via
 * \ref target_serial_use_uart_on_tdi_tdo, the serial task will rebind TDI/TDO at its next
 * polling iteration.  Intended to be invoked from \c swdptap_init when SWD reclaims control
 * of the TAP pins (SWD does not touch TDI/TDO, so they become available for UART again).
 */
void target_serial_tap_release_tdi_tdo(void);

/**
 * \brief Start FreeRTOS task(s) and DMA for USB↔UART bridging.
 */
void target_serial_init(void);

#endif /* MIOLINK_TARGET_SERIAL_H */
