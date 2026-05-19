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

#ifndef MIOLINK_TAP_PIO_H
#define MIOLINK_TAP_PIO_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/pio.h"

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define TAP_PIO_DMA_BUF_SIZE (16) /**< Size of the DMA buffer for PIO operations. */

/**
 * \brief Maximum number of SWD/JTAG shift ticks carried by one 32-bit PIO FIFO word.
 *
 * PIO TX/RX FIFO entries are 32-bit, so one entry drives at most 32 clock cycles. Per-call
 * \c clock_cycles / \c ticks arguments to the TAP shift helpers are bounded by this value;
 * longer sequences are split into multiple words by the caller.
 */
#define TAP_PIO_MAX_TICKS_PER_TRANSFER (32)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief State machine index for SWD and JTAG operations.
 */
typedef enum {
    TAP_PIO_SM_SWD = 0,
    TAP_PIO_SM_JTAG_TDI_TDO_SEQ = 0,
    TAP_PIO_SM_JTAG_TMS_SEQ = 1,
} tap_pio_sm_t;

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Busy-wait until the TX stall sticky flag is set for a state machine.
 *
 * Clears the sticky flag first, then polls until the TX FIFO is full and the SM is stalled on TX.
 *
 * \param[in] pio PIO block instance.
 * \param[in] sm  State machine index.
 */
static inline void tap_pio_wait_for_tx_stall(PIO pio, uint32_t sm)
{
    check_pio_param(pio);
    check_sm_param(sm);

    pio->fdebug = (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm));
    while ((pio->fdebug & (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0) {
        /* spin until PIO reports TX stall */
    }
}

/**
 * \brief Bypass input synchronizers for one GPIO used as a PIO input.
 *
 * \param[in] pio PIO block instance.
 * \param[in] pin Pin index (0-based) within the PIO GPIO mapping.
 */
static inline void tap_pio_disable_input_sync(PIO pio, uint32_t pin)
{
    check_pio_param(pio);

    pio->input_sync_bypass |= (1UL << pin);
}

/**
 * \brief Disable every state machine on the given PIO block.
 *
 * \param[in] pio PIO block instance.
 */
static inline void tap_pio_disable_all_machines(PIO pio)
{
    check_pio_param(pio);

    for (uint32_t i = 0; i < NUM_PIO_STATE_MACHINES; i++) {
        pio_sm_set_enabled(pio, i, false);
    }
}

/**
 * \brief Read PIO interrupt status register \c ints0.
 *
 * \param[in] pio PIO block instance.
 * \return Raw \c INTS0 value (pending IRQ flags for IRQ0 mapping).
 */
static inline uint32_t tap_pio_get_irq0_status(PIO pio)
{
    check_pio_param(pio);

    return pio->ints0;
}

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief DMA-transfer 32-bit words from RAM into a PIO TX FIFO (blocking).
 *
 * \param[in] pio          PIO block instance.
 * \param[in] sm           State machine index driving the TX FIFO.
 * \param[in] buffer_send  Source buffer; must not be \c NULL.
 * \param[in] data_amount  Number of 32-bit words to send; must be greater than 0 and at most \ref TAP_PIO_DMA_BUF_SIZE.
 */
void tap_pio_dma_send_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, const uint32_t data_amount);

/**
 * \brief DMA-send 32-bit words to PIO TX while draining the RX FIFO (blocking).
 *
 * \param[in]  pio                 PIO block instance.
 * \param[in]  sm                  State machine index.
 * \param[in]  buffer_send         Source buffer for TX; must not be \c NULL.
 * \param[out] buffer_recv         Optional buffer for captured RX words; may be \c NULL to discard.
 * \param[in]  data_amount         Number of 32-bit words to DMA to TX; must be greater than 0 and
 *                                 at most \ref TAP_PIO_DMA_BUF_SIZE.
 * \param[in]  data_amount_to_read Maximum number of RX words to store in \a buffer_recv.
 * \return Number of 32-bit words read from the RX FIFO. May exceed \a data_amount_to_read because the
 *         loop keeps draining the RX FIFO while the TX DMA channel is busy; surplus words are discarded
 *         (not written to \a buffer_recv).
 */
uint32_t tap_pio_dma_send_recv_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, uint32_t *buffer_recv,
    const uint32_t data_amount, const uint32_t data_amount_to_read);

/**
 * \brief DMA-transfer bytes from RAM into a PIO TX FIFO (blocking).
 *
 * \param[in] pio          PIO block instance.
 * \param[in] sm           State machine index.
 * \param[in] buffer_send  Source buffer; must not be \c NULL.
 * \param[in] data_amount  Byte count; must be greater than 0 and at most \ref TAP_PIO_DMA_BUF_SIZE.
 */
void tap_pio_dma_send_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, uint32_t data_amount);

/**
 * \brief DMA-send bytes to PIO TX while draining the RX FIFO (blocking).
 *
 * Each stored RX byte is taken from the high byte of the 32-bit word read from the SM RX FIFO.
 *
 * \param[in]  pio                 PIO block instance.
 * \param[in]  sm                  State machine index.
 * \param[in]  buffer_send         Source buffer for TX; must not be \c NULL.
 * \param[out] buffer_recv         Optional buffer for captured RX bytes; may be \c NULL to discard.
 * \param[in]  data_amount         Bytes to DMA to TX; must be greater than 0 and at most \ref TAP_PIO_DMA_BUF_SIZE.
 * \param[in]  data_amount_to_read Maximum number of RX bytes to store in \a buffer_recv.
 * \return Number of RX FIFO reads performed. May exceed \a data_amount_to_read because the loop keeps
 *         draining the RX FIFO while the TX DMA channel is busy; surplus bytes are discarded (not
 *         written to \a buffer_recv).
 */
uint32_t tap_pio_dma_send_recv_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, uint8_t *buffer_recv,
    uint32_t data_amount, uint32_t data_amount_to_read);

/**
 * \brief Set a state machine clock divider from a desired frequency and hardware limits.
 *
 * Clamps \a freq to the range from \c max_interface_freq/65536 through \a max_interface_freq, computes
 * integer/fractional divider, applies it, and restarts the SM clock divider.
 *
 * \param[in] pio                PIO block instance.
 * \param[in] sm                 State machine index.
 * \param[in] freq               Requested SM frequency (Hz), clamped as above.
 * \param[in] max_interface_freq Upper frequency bound used for divider math (Hz); also caps \a freq.
 * \return Actual frequency after clamping (before rounding to discrete divider steps), i.e. the clamped \a freq input.
 */
uint32_t tap_pio_set_sm_freq(PIO pio, uint32_t sm, uint32_t freq, uint32_t max_interface_freq);

#endif /* MIOLINK_TAP_PIO_H */
