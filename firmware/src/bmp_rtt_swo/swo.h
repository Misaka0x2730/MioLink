/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
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

#ifndef MIOLINK_SWO_H
#define MIOLINK_SWO_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "hardware/uart.h"
#include "FreeRTOS.h"

#include <stdlib.h>

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

/**
 * \brief Default baud when the host does not specify a SWO line rate.
 */
#define SWO_DEFAULT_BAUD (2250000U)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief Physical encoding selected for the SWO front-end.
 */
typedef enum swo_coding {
    swo_none,     /**< SWO is disabled. */
    swo_nrz_uart, /**< NRZ UART encoding (only mode supported on RP2040). */
} swo_coding_e;

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

extern swo_coding_e swo_current_mode; /**< Active SWO mode after the last \c swo_init / \c swo_deinit. */

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Configure UART SWO capture: pins, baud, optional DMA, ITM channel mask.
 *
 * \param[in] swo_mode            Must be \c swo_nrz_uart on this platform.
 * \param[in] baudrate            Line speed; \c 0 selects \ref SWO_DEFAULT_BAUD.
 * \param[in] itm_stream_bitmask  Non-zero to enable decoding for selected ITM stimulus ports.
 */
void swo_init(swo_coding_e swo_mode, uint32_t baudrate, uint32_t itm_stream_bitmask);

/**
 * \brief Tear down SWO UART and DMA; optionally release buffers.
 *
 * \param[in] deallocate Platform-specific deep cleanup flag.
 */
void swo_deinit(bool deallocate);

/**
 * \brief Current SWO UART baud rate.
 *
 * \return Active SWO baud rate, in bits per second, or \c 0 when SWO is not running.
 */
uint32_t swo_get_baudrate(void);

/**
 * \brief Limit which ITM stimulus ports are decoded and forwarded.
 *
 * \param[in] mask Bitmask passed from the monitor command handler.
 */
void traceswo_setmask(uint32_t mask);

/**
 * \brief Decode ITM/SWO from \a buf and emit on USB.
 *
 * \param[in] buf               Raw UART RX bytes.
 * \param[in] len               Length of \a buf.
 * \param[in] flush             Flush decoders / end of frame.
 * \param[in] drop_if_no_space  Drop data if USB TX is full.
 * \return \c false if output was dropped or stalled.
 */
bool traceswo_decode(const void *buf, uint16_t len, bool flush, bool drop_if_no_space);

/**
 * \brief Create the FreeRTOS “target_trace” task (claims SWO DMA channels).
 */
void traceswo_task_init(void);

#endif /* MIOLINK_SWO_H */
