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

#ifndef MIOLINK_DMA_EX_H
#define MIOLINK_DMA_EX_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/dma.h"

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define DMA_EX_CHANNEL_UNCLAIMED (-1) /**< Sentinel value for a DMA channel variable that has not been claimed yet. */

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Claim an unused DMA channel into \a dma_channel if it is currently unclaimed.
 *
 * If \c *dma_channel equals \ref DMA_EX_CHANNEL_UNCLAIMED, claims a free DMA channel via
 * \c dma_claim_unused_channel() (panicking if none is available) and stores the resulting
 * channel index in \c *dma_channel. Otherwise the value is left untouched.
 *
 * \param[in,out] dma_channel Pointer to the channel slot. Must be non-NULL.
 */
static inline void dma_ex_claim_channel_if_unclaimed(int *const dma_channel)
{
    assert(dma_channel != NULL);
    if (*dma_channel == DMA_EX_CHANNEL_UNCLAIMED) {
        *dma_channel = dma_claim_unused_channel(true);
    } else {
        check_dma_channel_param((uint)*dma_channel);
    }
}

/**
 * \brief Pointer to \c CHx_AL2_WRITE_ADDR_TRIG (write to start or reload chained write).
 *
 * \param[in] dma_channel DMA channel index (\c 0 … \c NUM_DMA_CHANNELS-1).
 * \return Pointer to the channel's \c AL2_WRITE_ADDR_TRIG register, suitable as a DMA write endpoint.
 */
static inline void *dma_ex_get_al2_write_addr_trig(const uint32_t dma_channel)
{
    check_dma_channel_param(dma_channel);
    return (void *)(&(dma_hw->ch[dma_channel].al2_write_addr_trig));
}

/**
 * \brief Remaining transfer count for \a dma_channel (\c TRANS_COUNT).
 *
 * \param[in] dma_channel DMA channel index (\c 0 … \c NUM_DMA_CHANNELS-1).
 * \return Number of transfers still pending on the channel.
 */
static inline uint32_t dma_ex_get_trans_count(const uint32_t dma_channel)
{
    check_dma_channel_param(dma_channel);
    return dma_hw->ch[dma_channel].transfer_count;
}

/**
 * \brief Enable/disable a channel via \c CTRL_TRIG or \c AL1_CTRL.
 *
 * \param[in] dma_channel Channel index.
 * \param[in] enabled     Desired run state.
 * \param[in] trigger     \c true: write \c CTRL_TRIG (may start transfer); \c false: alias \c AL1_CTRL only.
 */
static inline void dma_ex_set_channel_enabled(const uint32_t dma_channel, const bool enabled, const bool trigger)
{
    check_dma_channel_param(dma_channel);
    if (trigger) {
        hw_write_masked(&(dma_hw->ch[dma_channel].ctrl_trig), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
            DMA_CH0_CTRL_TRIG_EN_BITS);
    } else {
        hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
            DMA_CH0_CTRL_TRIG_EN_BITS);
    }
}

/**
 * \brief Disable the channel's interrupt on \a irq_index, abort it, and clear any pending IRQ flag.
 *
 * Masks the channel in \c INTE first so the ISR cannot observe the in-flight abort,
 * asserts \c CHAN_ABORT and spins on \c CTRL_TRIG.BUSY until the channel has retired
 * any pending transfer, then clears the corresponding bit in \c INTS so a stale
 * completion flag cannot fire once the IRQ is re-enabled.
 *
 * \param[in] dma_channel Channel index.
 * \param[in] irq_index   IRQ line index in the range \c 0 … \c NUM_DMA_IRQS-1
 *                    (\c 0 for \c DMA_IRQ_0, \c 1 for \c DMA_IRQ_1).
 */
static inline void dma_ex_channel_abort_and_disable_irq(const uint32_t dma_channel, const uint irq_index)
{
    check_dma_channel_param(dma_channel);
    dma_irqn_set_channel_enabled(irq_index, dma_channel, false);
    dma_channel_abort(dma_channel);
    dma_irqn_acknowledge_channel(irq_index, dma_channel);
}

/**
 * \brief Set \c CHAIN_TO so this channel chains into \a chain_to when complete.
 *
 * \param[in] dma_channel Channel whose \c CHAIN_TO field is updated.
 * \param[in] chain_to    Channel index to chain into when \a dma_channel completes.
 */
static inline void dma_ex_set_chain_to(const uint32_t dma_channel, const uint32_t chain_to)
{
    check_dma_channel_param(dma_channel);
    check_dma_channel_param(chain_to);
    hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), chain_to << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
        DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
}

#endif /* MIOLINK_DMA_EX_H */
