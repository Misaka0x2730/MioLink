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
 * Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/dma.h"

/**********************************************************************************************************************
 * Global Functions
 **********************************************************************************************************************/

/**
 * \brief Pointer to \c CHx_AL2_WRITE_ADDR_TRIG (write to start or reload chained write).
 *
 * \param dma_channel DMA channel index (\c 0 … \c NUM_DMA_CHANNELS-1).
 */
static inline void *dma_ex_get_al2_write_addr_trig(const uint32_t dma_channel)
{
	return (void *)(&(dma_hw->ch[dma_channel].al2_write_addr_trig));
}

/**
 * \brief Remaining transfer count for \a dma_channel (\c TRANS_COUNT).
 */
static inline uint32_t dma_ex_get_trans_count(const uint32_t dma_channel)
{
	return dma_hw->ch[dma_channel].transfer_count;
}

/**
 * \brief Enable/disable a channel via \c CTRL_TRIG or \c AL1_CTRL.
 *
 * \param dma_channel Channel index.
 * \param enabled     Desired run state.
 * \param trigger     \c true: write \c CTRL_TRIG (may start transfer); \c false: alias \c AL1_CTRL only.
 */
static inline void dma_ex_set_channel_enabled(const uint32_t dma_channel, const bool enabled, const bool trigger)
{
	if (trigger) {
		hw_write_masked(&(dma_hw->ch[dma_channel].ctrl_trig), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
			DMA_CH0_CTRL_TRIG_EN_BITS);
	} else {
		hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
			DMA_CH0_CTRL_TRIG_EN_BITS);
	}
}

/**
 * \brief Enable the DMA channel's interrupt on the given IRQ line.
 *
 * \param dma_channel Channel index.
 * \param irq_index   IRQ line index: \c 0 for \c DMA_IRQ_0, \c 1 for \c DMA_IRQ_1.
 */
static inline void dma_ex_channel_set_irq_enabled(const uint32_t dma_channel, const uint32_t irq_index)
{
	io_rw_32 * const reg = (irq_index == 0u) ? &dma_hw->inte0 : &dma_hw->inte1;
	hw_set_bits(reg, 1u << dma_channel);
}

/**
 * \brief Disable the DMA channel's interrupt on the given IRQ line.
 *
 * \param dma_channel Channel index.
 * \param irq_index   IRQ line index: \c 0 for \c DMA_IRQ_0, \c 1 for \c DMA_IRQ_1.
 */
static inline void dma_ex_channel_set_irq_disabled(const uint32_t dma_channel, const uint32_t irq_index)
{
	io_rw_32 * const reg = (irq_index == 0u) ? &dma_hw->inte0 : &dma_hw->inte1;
	hw_clear_bits(reg, 1u << dma_channel);
}

/**
 * \brief Set \c CHAIN_TO so this channel chains into \a chain_to when complete.
 */
static inline void dma_ex_set_chain_to(const uint32_t dma_channel, const uint32_t chain_to)
{
	hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), chain_to << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
		DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
}

#endif /* MIOLINK_DMA_EX_H */
