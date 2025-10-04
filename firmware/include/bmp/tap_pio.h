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

#include "general.h"

#include "hardware/pio.h"

#define TAP_PIO_SWD  (pio0)
#define TAP_PIO_JTAG (pio0)

typedef enum {
	TAP_PIO_SM_SWD = 0,
	TAP_PIO_SM_JTAG_TDI_TDO_SEQ = 0,
	TAP_PIO_SM_JTAG_TMS_SEQ = 1,
} tap_pio_sm_t;

#define TAP_PIO_DMA_BUF_SIZE (16)

static inline bool tap_pio_is_not_tx_stalled(PIO pio, uint32_t sm)
{
	check_pio_param(pio);
	check_sm_param(sm);

	pio->fdebug = (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm));
	return ((pio->fdebug & (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0);
}

static inline void tap_pio_wait_for_tx_stall(PIO pio, uint32_t sm)
{
	check_pio_param(pio);
	check_sm_param(sm);

	pio->fdebug = (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm));
	while ((pio->fdebug & (1UL << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0)
		;
}

static inline void tap_pio_disable_input_sync(PIO pio, uint32_t pin)
{
	check_pio_param(pio);

	pio->input_sync_bypass |= (1UL << pin);
}

static inline void tap_pio_disable_all_machines(PIO pio)
{
	check_pio_param(pio);

	for (uint32_t i = 0; i < NUM_PIO_STATE_MACHINES; i++) {
		pio_sm_set_enabled(pio, i, false);
	}
}

static inline uint32_t tap_pio_get_irq0_status(PIO pio)
{
	check_pio_param(pio);

	return pio->ints0;
}

void tap_pio_dma_send_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, const uint32_t data_amount);
uint32_t tap_pio_dma_send_recv_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, uint32_t *buffer_recv,
	const uint32_t data_amount, const uint32_t data_amount_to_read);
void tap_pio_dma_send_uint8(PIO pio, uint32_t sm, const uint8_t *buffer, uint32_t data_amount);
uint32_t tap_pio_dma_send_recv_uint8(PIO pio, uint32_t sm, const uint8_t *buffer, uint8_t *buffer_recv,
	uint32_t data_amount, uint32_t data_amount_to_read);
uint32_t tap_pio_set_sm_freq(PIO pio, uint32_t sm, uint32_t freq, uint32_t max_interface_freq);

#endif /* MIOLINK_TAP_PIO_H */
