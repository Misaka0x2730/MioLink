/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
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

#ifndef MIOLINK_SWO_H
#define MIOLINK_SWO_H

#include <stdlib.h>

#include "hardware/uart.h"

#include "FreeRTOS.h"

#define TRACESWO_UART      (uart0)
#define TRACESWO_UART_IRQ  (UART_IRQ_NUM(TRACESWO_UART))

#define TRACESWO_PIO       (pio1)
#define TRACESWO_PIO_IRQ   (PIO_IRQ_NUM(TRACESWO_PIO, 0))

#define TRACESWO_BUF_SIZE (1024)

/* Default line rate, used as default for a request without baudrate */
#define SWO_DEFAULT_BAUD 2250000U

typedef enum swo_coding {
	swo_none,
	swo_manchester,
	swo_nrz_uart,
} swo_coding_e;

extern swo_coding_e swo_current_mode;

/* Initialisation and deinitialisation functions (ties into command.c) */
void swo_init(swo_coding_e swo_mode, uint32_t baudrate, uint32_t itm_stream_bitmask);
void swo_deinit(bool deallocate);
uint32_t swo_get_baudrate(void);

/* Set bitmask of SWO channels to be decoded */
void traceswo_setmask(uint32_t mask);

/* Print decoded SWO packet on USB serial */
bool traceswo_decode(const void *buf, uint16_t len, bool flush, bool drop_if_no_space);

void traceswo_task_init(void);

BaseType_t traceswo_rx_dma_handler(void);

bool traceswo_uart_is_used(uart_inst_t *uart_instance);

#endif /* MIOLINK_SWO_H */