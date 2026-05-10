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

#ifndef MIOLINK_RP_UART_H
#define MIOLINK_RP_UART_H

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/clocks.h"
#include "hardware/uart.h"

/**********************************************************************************************************************
 * Global Definitions
 **********************************************************************************************************************/

#define RP_UART_INT_RX_BITS         (UART_UARTMIS_RXMIS_BITS)  /**< \c MIS mask: RX FIFO interrupt. */
#define RP_UART_INT_RX_TIMEOUT_BITS (UART_UARTMIS_RTMIS_BITS)   /**< \c MIS mask: RX timeout interrupt. */
#define RP_UART_INT_OE_BITS         (UART_UARTMIS_OEMIS_BITS)   /**< \c MIS mask: overrun error interrupt. */

/**********************************************************************************************************************
 * Global Functions
 **********************************************************************************************************************/

/**
 * \brief Enable UART DMA request lines for RX and/or TX.
 *
 * \param uart RP2040 UART instance.
 * \param rx   Enable RX DMA request.
 * \param tx   Enable TX DMA request.
 */
static inline void rp_uart_set_dma_req_enabled(uart_inst_t *uart, const bool rx, const bool tx)
{
	uart_get_hw(uart)->dmacr =
		(bool_to_bit(rx) << UART_UARTDMACR_RXDMAE_LSB) | (bool_to_bit(tx) << UART_UARTDMACR_TXDMAE_LSB);
}

/**
 * \brief Arm RX FIFO and/or RX timeout interrupts in \c IMSC.
 */
static inline void rp_uart_set_rx_and_timeout_irq_enabled(uart_inst_t *uart, const bool rx, const bool rx_timeout)
{
	hw_write_masked(&uart_get_hw(uart)->imsc,
		(bool_to_bit(rx) << UART_UARTIMSC_RXIM_LSB) | (bool_to_bit(rx_timeout) << UART_UARTIMSC_RTIM_LSB),
		UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
}

/**
 * \brief Enable or disable RX FIFO interrupt only.
 */
static inline void rp_uart_set_rx_irq_enabled(uart_inst_t *uart, const bool enabled)
{
	hw_write_masked(
		&uart_get_hw(uart)->imsc, (bool_to_bit(enabled) << UART_UARTIMSC_RXIM_LSB), UART_UARTIMSC_RXIM_BITS);
}

/**
 * \brief Clear RX interrupt sticky flag (\c ICR).
 */
static inline void rp_uart_clear_rx_irq_flag(uart_inst_t *uart)
{
	hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RXIC_BITS);
}

/**
 * \brief Clear RX timeout interrupt sticky flag (\c ICR).
 */
static inline void rp_uart_clear_rx_timeout_irq_flag(uart_inst_t *uart)
{
	hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RTIC_BITS);
}

/**
 * \brief Clear RX and RX-timeout interrupt flags together.
 */
static inline void rp_uart_clear_rx_and_rx_timeout_irq_flags(uart_inst_t *uart)
{
	hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS);
}

/**
 * \brief Enable or disable RX timeout interrupt in \c IMSC.
 */
static inline void rp_uart_set_rx_timeout_irq_enabled(uart_inst_t *uart, const bool enabled)
{
	hw_write_masked(
		&uart_get_hw(uart)->imsc, (bool_to_bit(enabled) << UART_UARTIMSC_RTIM_LSB), UART_UARTIMSC_RTIM_BITS);
}

/**
 * \brief Set RX/TX FIFO level thresholds for IRQ generation (\c IFLS).
 *
 * \param uart     UART instance.
 * \param rx_level \c UART_FIFO_* value for RX trigger level.
 * \param tx_level \c UART_FIFO_* value for TX trigger level.
 */
static inline void rp_uart_set_int_fifo_levels(uart_inst_t *uart, const uint8_t rx_level, const uint8_t tx_level)
{
	hw_write_masked(&uart_get_hw(uart)->ifls,
		(((uint32_t)rx_level) << UART_UARTIFLS_RXIFLSEL_LSB) | (((uint32_t)tx_level) << UART_UARTIFLS_TXIFLSEL_LSB),
		UART_UARTIFLS_RXIFLSEL_BITS | UART_UARTIFLS_TXIFLSEL_BITS);
}

/**
 * \brief Read masked interrupt status (\c MIS).
 */
static inline uint32_t rp_uart_get_int_status(uart_inst_t *uart)
{
	return uart_get_hw(uart)->mis;
}

/**
 * \brief Whether the RX FIFO is empty.
 */
static inline bool rp_uart_is_rx_fifo_empty(uart_inst_t *uart)
{
	return ((uart_get_hw(uart)->fr & UART_UARTFR_RXFE_BITS) != 0);
}

/**
 * \brief Read one byte from \c DR (non-blocking at register level).
 */
static inline uint8_t rp_uart_read(uart_inst_t *uart)
{
	return (uint8_t)(uart_get_hw(uart)->dr);
}

/**
 * \brief Address of \c DR for DMA write target (TX) or read source (RX).
 */
static inline void *rp_uart_get_dr_address(uart_inst_t *uart)
{
	return (void *)(&(uart_get_hw(uart)->dr));
}

/**
 * \brief Approximate baud rate from \c IBRD/\c FBRD and UART clock.
 */
static inline uint32_t rp_uart_get_baudrate(uart_inst_t *uart)
{
	const uint32_t ibrd = uart_get_hw(uart)->ibrd;
	const uint32_t fbrd = uart_get_hw(uart)->fbrd;
	return (4 * clock_get_hz(UART_CLOCK_NUM(uart))) / (64 * ibrd + fbrd);
}

/**
 * \brief UART busy transmitting (\c FR.BUSY).
 */
static inline bool rp_uart_is_transmitting(uart_inst_t *uart)
{
	return ((uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS) != 0);
}

#endif /* MIOLINK_RP_UART_H */
