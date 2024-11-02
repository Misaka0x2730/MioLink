/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

#include "general.h"
#include "platform.h"
#include "swd.h"

#include "hardware/pio.h"
#include "tap_pio_common.h"

static int pio_dma_channel = -1;
static dma_channel_config tx_config;

void tap_pio_common_dma_send(PIO pio, uint32_t sm, uint32_t *buffer, const uint8_t data_amount)
{
	assert(buffer != NULL);
	assert((data_amount > 0) && (data_amount <= PIO_BUFFER_SIZE));

	check_pio_param(pio);
	check_sm_param(sm);

	if (pio_dma_channel == -1)
	{
		pio_dma_channel = dma_claim_unused_channel(true);

		tx_config = dma_channel_get_default_config(pio_dma_channel);
		channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_32);
		channel_config_set_read_increment(&tx_config, true);
		channel_config_set_write_increment(&tx_config, false);

		channel_config_set_dreq(&tx_config, pio_get_dreq(TARGET_SWD_PIO, TARGET_SWD_PIO_SM, true));
	}

	dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer,
		data_amount, true);

	dma_channel_wait_for_finish_blocking(pio_dma_channel);
}