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

#ifndef MIOLINK_UART_EX_H
#define MIOLINK_UART_EX_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/clocks.h"
#include "hardware/uart.h"

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define RP_UART_INT_RX_BITS         (UART_UARTMIS_RXMIS_BITS) /**< \c MIS mask: RX FIFO interrupt. */
#define RP_UART_INT_RX_TIMEOUT_BITS (UART_UARTMIS_RTMIS_BITS) /**< \c MIS mask: RX timeout interrupt. */
#define RP_UART_INT_OE_BITS         (UART_UARTMIS_OEMIS_BITS) /**< \c MIS mask: overrun error interrupt. */

/**
 * \brief RX/TX FIFO depth of the RP2040 PL011 UART, in bytes.
 *
 * The Pico SDK does not expose this hardware fact as a constant; defined here so consumers can
 * derive byte-count thresholds from \ref uart_ex_rx_fifo_level_e / \ref uart_ex_tx_fifo_level_e.
 */
#define UART_EX_FIFO_DEPTH (32U)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief PL011 RX FIFO trigger levels.
 *
 * Selects when the UART asserts the RX interrupt (\c RXIM) based on RX FIFO occupancy. The PL011
 * supports five thresholds (1/8, 1/4, 1/2, 3/4, 7/8 of \ref UART_EX_FIFO_DEPTH); pick the value
 * that balances ISR rate against the headroom left before a hardware overrun for the expected
 * traffic pattern.
 *
 * Enumerator values match the PL011 \c UARTIFLS.RXIFLSEL register encoding (0..4) and are passed
 * directly into \ref uart_ex_set_int_fifo_levels.
 */
typedef enum uart_ex_rx_fifo_level {
    UART_EX_RX_FIFO_LEVEL_1_8 = 0, /**< Trigger at 1/8 FIFO. */
    UART_EX_RX_FIFO_LEVEL_1_4,     /**< Trigger at 1/4 FIFO. */
    UART_EX_RX_FIFO_LEVEL_1_2,     /**< Trigger at 1/2 FIFO. */
    UART_EX_RX_FIFO_LEVEL_3_4,     /**< Trigger at 3/4 FIFO. */
    UART_EX_RX_FIFO_LEVEL_7_8,     /**< Trigger at 7/8 FIFO. */
    UART_EX_RX_FIFO_LEVEL_COUNT,   /**< Number of valid trigger levels (sentinel; not a level). */
} uart_ex_rx_fifo_level_e;

/**
 * \brief PL011 TX FIFO trigger levels.
 *
 * Selects when the UART asserts the TX interrupt (\c TXIM) based on TX FIFO occupancy. The PL011
 * supports the same five thresholds as the RX side; the TX trigger fires when the FIFO occupancy
 * drops to or below the selected fraction.
 *
 * Enumerator values match the PL011 \c UARTIFLS.TXIFLSEL register encoding (0..4) and are passed
 * directly into \ref uart_ex_set_int_fifo_levels.
 */
typedef enum uart_ex_tx_fifo_level {
    UART_EX_TX_FIFO_LEVEL_1_8 = 0, /**< Trigger when FIFO occupancy ≤ 1/8. */
    UART_EX_TX_FIFO_LEVEL_1_4,     /**< Trigger when FIFO occupancy ≤ 1/4. */
    UART_EX_TX_FIFO_LEVEL_1_2,     /**< Trigger when FIFO occupancy ≤ 1/2. */
    UART_EX_TX_FIFO_LEVEL_3_4,     /**< Trigger when FIFO occupancy ≤ 3/4. */
    UART_EX_TX_FIFO_LEVEL_7_8,     /**< Trigger when FIFO occupancy ≤ 7/8. */
    UART_EX_TX_FIFO_LEVEL_COUNT,   /**< Number of valid trigger levels (sentinel; not a level). */
} uart_ex_tx_fifo_level_e;

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Enable UART DMA request lines for RX and/or TX.
 *
 * \param[in] uart RP2040 UART instance.
 * \param[in] rx   Enable RX DMA request.
 * \param[in] tx   Enable TX DMA request.
 */
static inline void uart_ex_set_dma_req_enabled(uart_inst_t *uart, const bool rx, const bool tx)
{
    uart_get_hw(uart)->dmacr =
        (bool_to_bit(rx) << UART_UARTDMACR_RXDMAE_LSB) | (bool_to_bit(tx) << UART_UARTDMACR_TXDMAE_LSB);
}

/**
 * \brief Arm RX FIFO and/or RX timeout interrupts in \c IMSC.
 *
 * \param[in] uart       RP2040 UART instance.
 * \param[in] rx         Enable RX FIFO interrupt.
 * \param[in] rx_timeout Enable RX timeout interrupt.
 */
static inline void uart_ex_set_rx_and_timeout_irq_enabled(uart_inst_t *uart, const bool rx, const bool rx_timeout)
{
    hw_write_masked(&uart_get_hw(uart)->imsc,
        (bool_to_bit(rx) << UART_UARTIMSC_RXIM_LSB) | (bool_to_bit(rx_timeout) << UART_UARTIMSC_RTIM_LSB),
        UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS);
}

/**
 * \brief Enable or disable RX FIFO interrupt only.
 *
 * \param[in] uart    RP2040 UART instance.
 * \param[in] enabled \c true to enable the RX FIFO interrupt; \c false to disable it.
 */
static inline void uart_ex_set_rx_irq_enabled(uart_inst_t *uart, const bool enabled)
{
    hw_write_masked(
        &uart_get_hw(uart)->imsc, (bool_to_bit(enabled) << UART_UARTIMSC_RXIM_LSB), UART_UARTIMSC_RXIM_BITS);
}

/**
 * \brief Clear RX interrupt sticky flag (\c ICR).
 *
 * \param[in] uart RP2040 UART instance.
 */
static inline void uart_ex_clear_rx_irq_flag(uart_inst_t *uart)
{
    hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RXIC_BITS);
}

/**
 * \brief Clear RX timeout interrupt sticky flag (\c ICR).
 *
 * \param[in] uart RP2040 UART instance.
 */
static inline void uart_ex_clear_rx_timeout_irq_flag(uart_inst_t *uart)
{
    hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RTIC_BITS);
}

/**
 * \brief Clear RX and RX-timeout interrupt flags together.
 *
 * \param[in] uart RP2040 UART instance.
 */
static inline void uart_ex_clear_rx_and_rx_timeout_irq_flags(uart_inst_t *uart)
{
    hw_set_bits(&uart_get_hw(uart)->icr, UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS);
}

/**
 * \brief Enable or disable RX timeout interrupt in \c IMSC.
 *
 * \param[in] uart    RP2040 UART instance.
 * \param[in] enabled \c true to enable the RX timeout interrupt; \c false to disable it.
 */
static inline void uart_ex_set_rx_timeout_irq_enabled(uart_inst_t *uart, const bool enabled)
{
    hw_write_masked(
        &uart_get_hw(uart)->imsc, (bool_to_bit(enabled) << UART_UARTIMSC_RTIM_LSB), UART_UARTIMSC_RTIM_BITS);
}

/**
 * \brief Set RX/TX FIFO level thresholds for IRQ generation (\c IFLS).
 *
 * \param[in] uart     UART instance.
 * \param[in] rx_level RX trigger level.
 * \param[in] tx_level TX trigger level.
 */
static inline void uart_ex_set_int_fifo_levels(
    uart_inst_t *uart, const uart_ex_rx_fifo_level_e rx_level, const uart_ex_tx_fifo_level_e tx_level)
{
    hw_write_masked(&uart_get_hw(uart)->ifls,
        (((uint32_t)rx_level) << UART_UARTIFLS_RXIFLSEL_LSB) | (((uint32_t)tx_level) << UART_UARTIFLS_TXIFLSEL_LSB),
        UART_UARTIFLS_RXIFLSEL_BITS | UART_UARTIFLS_TXIFLSEL_BITS);
}

/**
 * \brief Read masked interrupt status (\c MIS).
 *
 * \param[in] uart RP2040 UART instance.
 * \return Raw \c MIS register value (set of pending masked interrupt sources).
 */
static inline uint32_t uart_ex_get_int_status(uart_inst_t *uart)
{
    return uart_get_hw(uart)->mis;
}

/**
 * \brief Read one byte from \c DR (non-blocking at register level).
 *
 * \param[in] uart RP2040 UART instance.
 * \return The byte popped from the UART RX FIFO.
 */
static inline uint8_t uart_ex_read(uart_inst_t *uart)
{
    return (uint8_t)(uart_get_hw(uart)->dr);
}

/**
 * \brief Address of \c DR for DMA write target (TX) or read source (RX).
 *
 * \param[in] uart RP2040 UART instance.
 * \return Pointer to the UART \c DR register, suitable as a DMA endpoint.
 */
static inline void *uart_ex_get_dr_address(uart_inst_t *uart)
{
    return (void *)(&(uart_get_hw(uart)->dr));
}

/**
 * \brief Approximate baud rate from \c IBRD/\c FBRD and UART clock.
 *
 * \param[in] uart RP2040 UART instance.
 * \return Approximate baud rate, in bits per second, derived from current divider registers.
 */
static inline uint32_t uart_ex_get_baudrate(uart_inst_t *uart)
{
    const uint32_t ibrd = uart_get_hw(uart)->ibrd;
    const uint32_t fbrd = uart_get_hw(uart)->fbrd;
    return (4 * clock_get_hz(UART_CLOCK_NUM(uart))) / (64 * ibrd + fbrd);
}

/**
 * \brief UART busy transmitting (\c FR.BUSY).
 *
 * \param[in] uart RP2040 UART instance.
 * \return \c true if the UART transmitter is currently busy; \c false when the line is idle.
 */
static inline bool uart_ex_is_transmitting(uart_inst_t *uart)
{
    return ((uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS) != 0);
}

#endif /* MIOLINK_UART_EX_H */
