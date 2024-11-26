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

void tap_pio_common_dma_send_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, const uint32_t data_amount)
{
	assert(buffer_send != NULL);
	assert((data_amount > 0) && (data_amount <= PIO_BUFFER_SIZE));

	check_pio_param(pio);
	check_sm_param(sm);

	if (pio_dma_channel == -1) {
		pio_dma_channel = dma_claim_unused_channel(true);
	}

	dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_32);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

	dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

	dma_channel_wait_for_finish_blocking(pio_dma_channel);
}

uint32_t tap_pio_common_dma_send_recv_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, uint32_t *buffer_recv,
	const uint32_t data_amount, const uint32_t data_amount_to_read)
{
	assert(buffer_send != NULL);
	assert((data_amount > 0) && (data_amount <= PIO_BUFFER_SIZE));

	check_pio_param(pio);
	check_sm_param(sm);

	if (pio_dma_channel == -1) {
		pio_dma_channel = dma_claim_unused_channel(true);
	}

	dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_32);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

	dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

	uint32_t recv_data_amount = 0;
	while ((dma_channel_is_busy(pio_dma_channel)) || (recv_data_amount < data_amount_to_read)) {
		if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
			volatile const uint32_t read_value = pio_sm_get_blocking(pio, sm);
			if ((buffer_recv != NULL) && (recv_data_amount < data_amount_to_read)) {
				buffer_recv[recv_data_amount] = read_value;
			}
			recv_data_amount++;
		}
	}

	__compiler_memory_barrier();

	return recv_data_amount;
}

void tap_pio_common_dma_send_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, const uint32_t data_amount)
{
	assert(buffer_send != NULL);
	assert((data_amount > 0) && (data_amount <= PIO_BUFFER_SIZE));

	check_pio_param(pio);
	check_sm_param(sm);

	if (pio_dma_channel == -1) {
		pio_dma_channel = dma_claim_unused_channel(true);
	}

	dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

	dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

	dma_channel_wait_for_finish_blocking(pio_dma_channel);
}

uint32_t tap_pio_common_dma_send_recv_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, uint8_t *buffer_recv,
	const uint32_t data_amount, const uint32_t data_amount_to_read)
{
	assert(buffer_send != NULL);
	assert((data_amount > 0) && (data_amount <= PIO_BUFFER_SIZE));

	check_pio_param(pio);
	check_sm_param(sm);

	if (pio_dma_channel == -1) {
		pio_dma_channel = dma_claim_unused_channel(true);
	}

	dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

	dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

	uint32_t recv_data_amount = 0;
	while ((dma_channel_is_busy(pio_dma_channel)) || (recv_data_amount < data_amount_to_read)) {
		if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
			volatile const uint8_t read_value = (uint8_t)((pio_sm_get_blocking(pio, sm) >> 24) & 0xFF);
			if ((buffer_recv != NULL) && (recv_data_amount < data_amount_to_read)) {
				buffer_recv[recv_data_amount] = read_value;
			}
			recv_data_amount++;
		}
	}

	__compiler_memory_barrier();

	return recv_data_amount;
}