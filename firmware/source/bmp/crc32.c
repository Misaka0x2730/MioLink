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

#include "hardware/dma.h"

#include "target.h"
#include "gdb_if.h"

static int crc_dma_channel = -1;

static bool rp2040_crc32(target_s *const target, uint32_t *const result, const uint32_t base, const size_t len)
{
	uint8_t bytes[2048U]; /* ADIv5 MEM-AP AutoInc range */

	uint32_t last_time = platform_time_ms();
	const size_t adjusted_len = len & ~3U;
	uint32_t crc = 0xFFFFFFFF;

	if (crc_dma_channel == -1) {
		crc_dma_channel = dma_claim_unused_channel(true);
	}

	dma_channel_config dma_config = dma_channel_get_default_config(crc_dma_channel);
	channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
	channel_config_set_read_increment(&dma_config, true);
	channel_config_set_write_increment(&dma_config, false);

	channel_config_set_sniff_enable(&dma_config, true);
	dma_sniffer_enable(crc_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC32, true);
	dma_sniffer_set_byte_swap_enabled(false);
	dma_sniffer_set_output_reverse_enabled(false);

	for (size_t offset = 0; offset < adjusted_len; offset += sizeof(bytes)) {
		const uint32_t actual_time = platform_time_ms();
		if (actual_time > last_time + 1000U) {
			last_time = actual_time;
			gdb_if_putchar(0, true);
		}
		const size_t read_len = MIN(sizeof(bytes), adjusted_len - offset);
		if (target_mem32_read(target, bytes, base + offset, read_len)) {
			DEBUG_ERROR("%s: error around address 0x%08" PRIx32 "\n", __func__, (uint32_t)(base + offset));
			return false;
		}

		uint32_t dummy_dst = 0;

		dma_sniffer_set_data_accumulator(crc);
		dma_channel_configure(crc_dma_channel, &dma_config, &dummy_dst, bytes, read_len / 4, true);

		dma_channel_wait_for_finish_blocking(crc_dma_channel);
		crc = dma_sniffer_get_data_accumulator();
	}

	const size_t remainder = len - adjusted_len;
	if (remainder) {
		if (target_mem32_read(target, bytes, base + adjusted_len, remainder)) {
			DEBUG_ERROR("%s: error around address 0x%08" PRIx32 "\n", __func__, (uint32_t)(base + adjusted_len));
			return false;
		}
		for (size_t offset = 0; offset < remainder; ++offset) {
			crc ^= bytes[offset] << 24U;
			for (size_t i = 0; i < 8U; i++) {
				if (crc & 0x80000000U)
					crc = (crc << 1U) ^ 0x4c11db7U;
				else
					crc <<= 1U;
			}
		}
	}
	*result = crc;
	return true;
}

/* Shim to dispatch host-specific implementation (and keep the `__func__` meaningful) */
bool bmd_crc32(target_s *const target, uint32_t *const result, const uint32_t base, const size_t len)
{
#ifndef DEBUG_INFO_IS_NOOP
	const uint32_t start_time = platform_time_ms();
#endif
	const bool status = rp2040_crc32(target, result, base, len);

#ifndef DEBUG_INFO_IS_NOOP
	const uint32_t end_time = platform_time_ms();
	const uint32_t time_elapsed = end_time - start_time;
	DEBUG_INFO("%s: 0x%08" PRIx32 "+%" PRIu32 " -> %" PRIu32 "ms", __func__, base, (uint32_t)len, time_elapsed);
	if (len >= 512U) {
		const uint32_t speed = len * 1000U / time_elapsed / 1024U;
		DEBUG_INFO(", %" PRIu32 " KiB/s", speed);
	}
	DEBUG_INFO("\n");
#endif
	return status;
}