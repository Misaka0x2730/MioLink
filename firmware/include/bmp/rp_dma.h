/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2024 Dmitry Rezvanov <gareth@blacksphere.co.nz>
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

#ifndef MIOLINK_RP_DMA_H
#define MIOLINK_RP_DMA_H

#include "general.h"
#include "hardware/dma.h"

static inline void *rp_dma_get_al2_write_addr_trig(const uint32_t dma_channel)
{
	return (void *)(&(dma_hw->ch[dma_channel].al2_write_addr_trig));
}

static inline uint32_t rp_dma_get_trans_count(const uint32_t dma_channel)
{
	return dma_hw->ch[dma_channel].transfer_count;
}

static inline void rp_dma_set_channel_enabled(const uint32_t dma_channel, const bool enabled, const bool trigger)
{
	if (trigger) {
		hw_write_masked(&(dma_hw->ch[dma_channel].ctrl_trig), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
			DMA_CH0_CTRL_TRIG_EN_BITS);
	} else {
		hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), (bool_to_bit(enabled) << DMA_CH0_CTRL_TRIG_EN_LSB),
			DMA_CH0_CTRL_TRIG_EN_BITS);
	}
}

static inline void rp_dma_set_chain_to(const uint32_t dma_channel, const uint32_t chain_to)
{
	hw_write_masked(&(dma_hw->ch[dma_channel].al1_ctrl), chain_to << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
		DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
}

#endif //MIOLINK_RP_DMA_H