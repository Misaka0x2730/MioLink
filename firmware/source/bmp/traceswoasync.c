/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011 Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022-2024 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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

#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "rp_uart.h"
#include "rp_dma.h"

#include "FreeRTOS.h"
#include "atomic.h"
#include "timers.h"
#include "task.h"

#include "tusb.h"

#include "platform.h"
#include "usb_serial.h"

#include "swo.h"
#include "gdb_packet.h"

#include "hardware/pio.h"
#include "tap_pio_common.h"
#include "traceswo_manchester.pio.h"

#define TRACESWO_UART_RX_INT_FIFO_LEVEL (16)

#define TRACESWO_RX_DMA_TOTAL_BUFFERS_SIZE (16 * 1024)
#define TRACESWO_RX_DMA_NUMBER_OF_BUFFERS  (32)
#define TRACESWO_RX_DMA_BUFFER_SIZE \
	(TRACESWO_RX_DMA_TOTAL_BUFFERS_SIZE / TRACESWO_RX_DMA_NUMBER_OF_BUFFERS)

#define TRACESWO_RX_DMA_DROP_BUFFER_THRESHOLD (TRACESWO_RX_DMA_NUMBER_OF_BUFFERS / 3)

#if (TRACESWO_RX_DMA_NUMBER_OF_BUFFERS < 4)
#error "TRACESWO_RX_DMA_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define TRACESWO_RX_DMA_BAUDRATE_THRESHOLD (38400)
#define TRACESWO_RX_DMA_MIN_TIMEOUT        (50)
#define TRACESWO_RX_DMA_MAX_TIMEOUT        (200)
#define TRACESWO_RX_INT_PIO_TIMEOUT        (100)

#define TRACESWO_PIO_CLOCKS_IN_ONE_BIT (12)

#define TRACESWO_TASK_NOTIFY_WAIT_PERIOD (portMAX_DELAY)

#define TRACESWO_VENDOR_INTERFACE (0)

#define TRACESWO_DECODE_THRESHOLD (64)

#define TRACESWO_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define TRACESWO_TASK_STACK_SIZE    (512)

static uint8_t rx_buf[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS][TRACESWO_RX_DMA_BUFFER_SIZE] = {0};
static uint32_t rx_int_buf_pos = 0;
static xTimerHandle rx_timeout_timer;
static uint32_t rx_dma_buffer_full_mask = 0;
static uint32_t rx_dma_current_buffer = 0;
static bool rx_use_dma = false;
static int rx_dma_channel = -1;
static int rx_dma_ctrl_channel = -1;
static uint32_t rx_dma_next_buffer_to_send = 0;
static bool rx_ongoing = false;

/* Current SWO decoding mode being used */
swo_coding_e swo_current_mode = swo_none;

static struct {
	uint8_t *address;
} rx_dma_ctrl_block_info[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS + 1]
	__attribute__((aligned(TRACESWO_RX_DMA_NUMBER_OF_BUFFERS * sizeof(uint32_t))));

typedef enum {
	TRACESWO_PIO_SM = 0,
} traceswo_pio_sm;


static uint32_t traceswo_pio_baudrate = 0;

static bool traceswo_decoding = false;

static TaskHandle_t traceswo_task;

static void traceswo_rx_uart_handler(void);
static void traceswo_rx_pio_handler(void);

void traceswo_update_led(void)
{
	platform_set_serial_state(rx_ongoing);
}

static bool traceswo_send_to_usb(uint8_t *data, const size_t len, const bool flush, const bool allow_drop_buffer)
{
	bool result = false;
	const uint32_t write_available = tud_vendor_n_write_available(TRACESWO_VENDOR_INTERFACE);

	if (write_available < len) {
		if (allow_drop_buffer) {
			tud_vendor_n_write(TRACESWO_VENDOR_INTERFACE, data, write_available);
			result = true;
		}
	} else {
		result = (tud_vendor_n_write(TRACESWO_VENDOR_INTERFACE, data, len) == len);
	}

	if (result && flush) {
		tud_vendor_n_flush(TRACESWO_VENDOR_INTERFACE);
	}

	return result;
}

static void dma_rx_init(swo_coding_e swo_mode, const uint32_t baudrate)
{
	assert(swo_mode != swo_none);

	if (swo_mode == swo_nrz_uart)
	{
		rp_uart_set_dma_req_enabled(TRACESWO_UART, false, false);
		rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, false, false);
		rp_uart_set_int_fifo_levels(TRACESWO_UART, 0, 0);
	}
	else if (swo_mode == swo_manchester)
	{
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), false);
	}

	dma_channel_set_irq0_enabled(rx_dma_ctrl_channel, false);
	dma_channel_abort(rx_dma_ctrl_channel);
	dma_channel_acknowledge_irq0(rx_dma_ctrl_channel);

	dma_channel_set_irq0_enabled(rx_dma_channel, false);
	dma_channel_abort(rx_dma_channel);
	dma_channel_acknowledge_irq0(rx_dma_channel);

	dma_channel_config rx_ctrl_config = dma_channel_get_default_config(rx_dma_ctrl_channel);
	channel_config_set_transfer_data_size(&rx_ctrl_config, DMA_SIZE_32);
	channel_config_set_read_increment(&rx_ctrl_config, true);
	channel_config_set_write_increment(&rx_ctrl_config, false);
	channel_config_set_high_priority(&rx_ctrl_config, true);
	channel_config_set_ring(&rx_ctrl_config, false, 7);

	dma_channel_configure(rx_dma_ctrl_channel, &rx_ctrl_config,
		rp_dma_get_al2_write_addr_trig(rx_dma_channel),
		rx_dma_ctrl_block_info, 1, false);

	dma_channel_config rx_config = dma_channel_get_default_config(rx_dma_channel);
	channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
	channel_config_set_read_increment(&rx_config, false);
	channel_config_set_write_increment(&rx_config, true);
	channel_config_set_high_priority(&rx_config, true);
	channel_config_set_chain_to(&rx_config, rx_dma_ctrl_channel);

	if (swo_mode == swo_nrz_uart)
	{
		channel_config_set_dreq(&rx_config, uart_get_dreq(TRACESWO_UART, false));
		dma_channel_configure(rx_dma_channel, &rx_config, rx_buf, rp_uart_get_dr_address(TRACESWO_UART),
			TRACESWO_RX_DMA_BUFFER_SIZE, false);
	}
	else if (swo_mode == swo_manchester)
	{
		channel_config_set_dreq(&rx_config, pio_get_dreq(TRACESWO_PIO, TRACESWO_PIO_SM, false));
		dma_channel_configure(rx_dma_channel, &rx_config, rx_buf, (void *)&(TRACESWO_PIO->rxf[0]) + 3,
			TRACESWO_RX_DMA_BUFFER_SIZE, false);
	}

	rx_use_dma = true;

	rp_dma_set_channel_enabled(rx_dma_channel, false, false);
	dma_channel_set_irq0_enabled(rx_dma_channel, true);

	dma_channel_set_read_addr(rx_dma_ctrl_channel, (void *)rx_dma_ctrl_block_info, true);

	/* Calculate timer period - time to fill 2 rx buffers */
	uint32_t timer_period = (TRACESWO_RX_DMA_BUFFER_SIZE * 2 * 1000); /* 1000 - because we need milliseconds */
	timer_period /= (baudrate / 10); /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
	if (timer_period < TRACESWO_RX_DMA_MIN_TIMEOUT) {
		timer_period = TRACESWO_RX_DMA_MIN_TIMEOUT;
	} else if (timer_period > TRACESWO_RX_DMA_MAX_TIMEOUT) {
		timer_period = TRACESWO_RX_DMA_MAX_TIMEOUT;
	}

	xTimerChangePeriod(rx_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

	if (swo_mode == swo_nrz_uart)
	{
		rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);
	}
	else if (swo_mode == swo_manchester)
	{
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), true);
	}
}

static BaseType_t rx_dma_start_receiving(void)
{
	assert(rx_ongoing == false);

	rx_ongoing = true;

	if (swo_current_mode == swo_nrz_uart) {
		rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, false, false);
		rp_uart_clear_rx_and_rx_timeout_irq_flags(TRACESWO_UART);
		rp_uart_set_dma_req_enabled(TRACESWO_UART, true, false);

		rp_dma_set_channel_enabled(rx_dma_channel, true, false);
	} else if (swo_current_mode == swo_manchester)
	{
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), false);
		rp_dma_set_channel_enabled(rx_dma_channel, true, true);
	}

	dma_channel_acknowledge_irq0(rx_dma_channel);
	dma_channel_set_irq0_enabled(rx_dma_channel,  true);

	BaseType_t higher_priority_task_woken = pdFALSE;

	xTimerResetFromISR(rx_timeout_timer, &higher_priority_task_woken);
	return higher_priority_task_woken;
}

static void rx_dma_process_buffers(void)
{
	xTimerReset(rx_timeout_timer, 0);

	while (1) {
		const uint32_t buffer_state = rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1u << rx_dma_next_buffer_to_send);
		const bool allow_drop_buffer = (__builtin_popcount(buffer_state) >= TRACESWO_RX_DMA_DROP_BUFFER_THRESHOLD);
		const uint32_t data_len = sizeof(rx_buf[rx_dma_next_buffer_to_send]);
		if (buffer_state & buffer_bit) {
			if (traceswo_decoding) {
				if (traceswo_decode(rx_buf[rx_dma_next_buffer_to_send], data_len, false, allow_drop_buffer)) {
					Atomic_AND_u32(&rx_dma_buffer_full_mask, ~buffer_bit);
					if (++rx_dma_next_buffer_to_send >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
						rx_dma_next_buffer_to_send = 0;
					}
				} else {
					xTimerReset(rx_timeout_timer, 0);
					vTaskDelay(pdMS_TO_TICKS(1));
					continue;
				}
			} else if (traceswo_send_to_usb(
						   rx_buf[rx_dma_next_buffer_to_send], data_len, false, allow_drop_buffer)) {
				Atomic_AND_u32(&rx_dma_buffer_full_mask, ~buffer_bit);
				if (++rx_dma_next_buffer_to_send >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
					rx_dma_next_buffer_to_send = 0;
				}
			}
		} else {
			break;
		}
	}
}

static void rx_timeout_callback(TimerHandle_t xTimer)
{
	(void)(xTimer);
	if (rx_use_dma) {
		xTaskNotify(traceswo_task, USB_SERIAL_DATA_RX_TIMEOUT, eSetBits);
	} else if (swo_current_mode == swo_manchester) {
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), false);
		xTaskNotify(traceswo_task, USB_SERIAL_DATA_RX_FLUSH, eSetBits);
	}
}

static bool rx_dma_finish_receiving(void)
{
	assert(rx_ongoing != false);

	if (swo_current_mode == swo_nrz_uart) {
		rp_uart_set_dma_req_enabled(TRACESWO_UART, false, false);
	} else if (swo_current_mode == swo_manchester)
	{
		pio_sm_set_enabled(TRACESWO_PIO, TRACESWO_PIO_SM, false);
	}

	dma_channel_set_irq0_enabled(rx_dma_ctrl_channel, false);
	dma_channel_abort(rx_dma_ctrl_channel);
	dma_channel_acknowledge_irq0(rx_dma_ctrl_channel);

	const uint32_t current_buffer = rx_dma_current_buffer;

	if (++rx_dma_current_buffer >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
		rx_dma_current_buffer = 0;
	}

	xTimerStop(rx_timeout_timer, pdMS_TO_TICKS(0));

	rx_ongoing = false;

	const uint32_t remaining = rp_dma_get_trans_count(rx_dma_channel);
	const uint32_t data_in_buffer = sizeof(rx_buf[0]) - remaining;

	dma_channel_set_irq0_enabled(rx_dma_channel, false);
	dma_channel_abort(rx_dma_channel);
	dma_channel_acknowledge_irq0(rx_dma_channel);

	dma_channel_set_read_addr(rx_dma_ctrl_channel, (void *)(rx_dma_ctrl_block_info + rx_dma_current_buffer), true);

	if (swo_current_mode == swo_nrz_uart) {
		rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);
	} else if (swo_current_mode == swo_manchester)
	{
		rp_dma_set_channel_enabled(rx_dma_channel, false, false);

		pio_sm_set_enabled(TRACESWO_PIO, TRACESWO_PIO_SM, true);
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), true);
	}

	while (1) {
		const uint32_t buffer_state = rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1u << rx_dma_next_buffer_to_send);
		if (buffer_state & buffer_bit) {
			if (traceswo_decoding) {
				traceswo_decode(rx_buf[rx_dma_next_buffer_to_send],
					sizeof(rx_buf[rx_dma_next_buffer_to_send]), false, true);
			} else {
				traceswo_send_to_usb(rx_buf[rx_dma_next_buffer_to_send],
					sizeof(rx_buf[rx_dma_next_buffer_to_send]), false, true);
			}

			Atomic_AND_u32(&rx_dma_buffer_full_mask, ~buffer_bit);
			if (++rx_dma_next_buffer_to_send >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
				rx_dma_next_buffer_to_send = 0;
			}
		} else {
			break;
		}
	}

	if ((current_buffer + 1) >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
		rx_dma_next_buffer_to_send = 0;
	} else {
		rx_dma_next_buffer_to_send = current_buffer + 1;
	}

	if (traceswo_decoding) {
		traceswo_decode(rx_buf[current_buffer], data_in_buffer, true, true);
	} else {
		traceswo_send_to_usb(rx_buf[current_buffer], data_in_buffer, true, true);
	}

	return false;
}

static void rx_int_init(swo_coding_e swo_mode, const uint32_t baudrate)
{
	rx_use_dma = false;

	if (swo_mode == swo_nrz_uart)
	{
		/* Set RX FIFO level to 1/2 */
		rp_uart_set_int_fifo_levels(TRACESWO_UART, 2, 0);
		rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);
	}
	else if (swo_mode == swo_manchester)
	{
		xTimerChangePeriod(rx_timeout_timer, pdMS_TO_TICKS(TRACESWO_RX_INT_PIO_TIMEOUT), portMAX_DELAY);

		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), true);
	}
}

static void rx_int_process(void)
{
	if (rx_int_buf_pos > 0) {
		if ((traceswo_decoding) && (rx_int_buf_pos >= TRACESWO_DECODE_THRESHOLD)) {
			/* write decoded swo packets to the uart port */
			traceswo_decode(rx_buf, rx_int_buf_pos, false, true);

			rx_int_buf_pos = 0;
		} else if (traceswo_send_to_usb((uint8_t *)(rx_buf), rx_int_buf_pos, false, true) != false) {
			rx_int_buf_pos = 0;
		}

		rx_ongoing = true;
	}

	if (swo_current_mode == swo_nrz_uart) {
		rp_uart_set_rx_irq_enabled(TRACESWO_UART, true);
	} else if (swo_current_mode == swo_manchester)
	{
		xTimerReset(rx_timeout_timer, 0);
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), true);
	}
}

static void rx_int_finish(void)
{
	if (rx_int_buf_pos > 0) {
		traceswo_send_to_usb((uint8_t *)(rx_buf), rx_int_buf_pos, false, true);
		rx_int_buf_pos = 0;
	}

	if (swo_current_mode == swo_nrz_uart) {
		while (rp_uart_is_rx_fifo_empty(TRACESWO_UART) == false) {
			((uint8_t *)rx_buf)[rx_int_buf_pos++] = rp_uart_read(TRACESWO_UART);

			if (rx_int_buf_pos >= sizeof(rx_buf)) {
				rx_int_buf_pos = 0;
			}
		}
	} else if (swo_current_mode == swo_manchester)
	{
		xTimerStop(rx_timeout_timer, pdMS_TO_TICKS(0));

		while (1)
		{
			if (pio_sm_get_rx_fifo_level(TRACESWO_PIO, TRACESWO_PIO_SM) == 0)
			{
				break;
			}

			((uint8_t *)rx_buf)[rx_int_buf_pos++] = pio_sm_get(TRACESWO_PIO, TRACESWO_PIO_SM);
			if (rx_int_buf_pos >= sizeof(rx_buf)) {
				rx_int_buf_pos = 0;
			}
		}
	}

	if (rx_int_buf_pos > 0) {
		if (traceswo_decoding) {
			/* write decoded swo packets to the uart port */
			traceswo_decode(rx_buf, rx_int_buf_pos, true, true);
		} else {
			traceswo_send_to_usb((uint8_t *)(rx_buf), rx_int_buf_pos, true, true);
		}

		rx_int_buf_pos = 0;
	}

	rx_ongoing = false;

	if (swo_current_mode == swo_nrz_uart) {
		rp_uart_set_rx_timeout_irq_enabled(TRACESWO_UART, true);
	} else if (swo_current_mode == swo_manchester)
	{
		pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), true);
	}

}

void swo_init(swo_coding_e swo_mode, uint32_t baudrate, uint32_t itm_stream_bitmask)
{
	usb_serial_uart_release(TRACESWO_UART);

	portENTER_CRITICAL();

	swo_deinit(false);

	if (baudrate == 0) {
		baudrate = SWO_DEFAULT_BAUD;
	}

	if (swo_mode == swo_nrz_uart)
	{
		const platform_target_pins_t *target_pins = platform_get_target_pins();

		/* Re-initialize UART */
		gpio_set_function(target_pins->tdo, GPIO_FUNC_UART);
		gpio_set_function(target_pins->tdi, GPIO_FUNC_SIO);

		irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_UART_IRQ);
		assert(current_handler == NULL);

		irq_set_exclusive_handler(TRACESWO_UART_IRQ, traceswo_rx_uart_handler);

		uart_init(TRACESWO_UART, baudrate);
	}
	else if (swo_mode == swo_manchester)
	{
		const platform_target_pins_t *target_pins = platform_get_target_pins();

		/* Re-initialize PIO */
		gpio_init(target_pins->tdo);
		pio_gpio_init(TRACESWO_PIO, target_pins->tdo);

		pio_sm_set_pindirs_with_mask(TRACESWO_PIO, TRACESWO_PIO_SM, 0, (1 << target_pins->tdo));
		pio_sm_set_pins_with_mask(TRACESWO_PIO, TRACESWO_PIO_SM, 0, (1 << target_pins->tdo));

		pio_clear_instruction_memory(TRACESWO_PIO);
		pio_add_program_at_offset(TRACESWO_PIO, &traceswo_manchester_program, traceswo_manchester_program.origin);

		pio_sm_config traceswo_program_config = traceswo_manchester_program_get_default_config(traceswo_manchester_program.origin);
		sm_config_set_in_pins(&traceswo_program_config, target_pins->tdo);
		sm_config_set_jmp_pin(&traceswo_program_config, target_pins->tdo);
		sm_config_set_in_shift(&traceswo_program_config, true, true, 8);
		sm_config_set_fifo_join(&traceswo_program_config, PIO_FIFO_JOIN_RX);

		tap_pio_common_disable_input_sync(TARGET_SWD_PIO, target_pins->tdo);

		irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_PIO_IRQ);
		assert(current_handler == NULL);

		irq_set_exclusive_handler(TRACESWO_PIO_IRQ, traceswo_rx_pio_handler);

		pio_sm_init(TRACESWO_PIO, TRACESWO_PIO_SM, traceswo_manchester_program.origin, &traceswo_program_config);
		pio_sm_set_enabled(TRACESWO_PIO, TRACESWO_PIO_SM, true);

		traceswo_pio_baudrate = (tap_pio_common_set_sm_freq(TRACESWO_PIO, TRACESWO_PIO_SM, (baudrate / 2) * TRACESWO_PIO_CLOCKS_IN_ONE_BIT, clock_get_hz(clk_sys))) * 2 / TRACESWO_PIO_CLOCKS_IN_ONE_BIT;
	}

	if (baudrate >= TRACESWO_RX_DMA_BAUDRATE_THRESHOLD) {
		dma_rx_init(swo_mode, baudrate);
	} else {
		rx_int_init(swo_mode, baudrate);
	}

	memset(rx_buf, 0x00, sizeof(rx_buf));

	const uint32_t irq_num = (swo_mode == swo_nrz_uart) ? TRACESWO_UART_IRQ : TRACESWO_PIO_IRQ;
	irq_set_enabled(irq_num, true);

	traceswo_setmask(itm_stream_bitmask);
	traceswo_decoding = itm_stream_bitmask != 0;

	swo_current_mode = swo_mode;

	portEXIT_CRITICAL();

	gdb_outf("Baudrate: %" PRIu32 " ", swo_get_baudrate());
}

void swo_deinit(bool deallocate)
{
	(void)(deallocate);

	if (swo_current_mode != swo_none) {
		portENTER_CRITICAL();

		dma_channel_set_irq0_enabled(rx_dma_ctrl_channel, false);
		dma_channel_abort(rx_dma_ctrl_channel);
		dma_channel_acknowledge_irq0(rx_dma_ctrl_channel);

		dma_channel_set_irq0_enabled(rx_dma_channel, false);
		dma_channel_abort(rx_dma_channel);
		dma_channel_acknowledge_irq0(rx_dma_channel);

		const uint32_t irq_num = (swo_current_mode == swo_nrz_uart) ? TRACESWO_UART_IRQ : TRACESWO_PIO_IRQ;

		irq_handler_t current_handler = irq_get_exclusive_handler(irq_num);
		irq_set_enabled(irq_num, false);

		if (current_handler != NULL) {
			irq_remove_handler(irq_num, current_handler);
		}

		if (swo_current_mode == swo_nrz_uart)
		{
			uart_deinit(TRACESWO_UART);
		}
		else if (swo_current_mode == swo_manchester)
		{
			tap_pio_common_disable_all_machines(TRACESWO_PIO);
			pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), false);

			pio_clear_instruction_memory(TRACESWO_PIO);
		}

		rx_int_buf_pos = 0;
		rx_ongoing = false;

		portEXIT_CRITICAL();
	}

	swo_current_mode = swo_none;
}

bool traceswo_uart_is_used(uart_inst_t *uart_instance)
{
	return ((swo_current_mode == swo_nrz_uart) && (uart_instance == TRACESWO_UART));
}

uint32_t swo_get_baudrate(void)
{
	if (swo_current_mode == swo_nrz_uart) {
		return rp_uart_get_baudrate(TRACESWO_UART);
	} else if (swo_current_mode == swo_manchester) {
		return traceswo_pio_baudrate;
	}

	return 0;
}

_Noreturn static void traceswo_thread(void *params);

void traceswo_task_init(void)
{
#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(traceswo_thread, "target_trace", TRACESWO_TASK_STACK_SIZE, NULL,
		PLATFORM_PRIORITY_NORMAL, TRACESWO_TASK_CORE_AFFINITY, &traceswo_task);
#else
	const BaseType_t result = xTaskCreate(
		traceswo_thread, "target_trace", TRACESWO_TASK_STACK_SIZE, NULL, PLATFORM_PRIORITY_NORMAL, &traceswo_task);
#endif
	assert(result == pdPASS);
}

_Noreturn static void traceswo_thread(void *params)
{
	uint32_t notificationValue = 0;

	rx_timeout_timer =
		xTimerCreate("TRACE_RX_TIMEOUT", pdMS_TO_TICKS(TRACESWO_RX_DMA_MAX_TIMEOUT), pdFALSE, NULL, rx_timeout_callback);

	rx_dma_channel = dma_claim_unused_channel(true);
	rx_dma_ctrl_channel = dma_claim_unused_channel(true);

	memset(&rx_dma_ctrl_block_info, 0, sizeof(rx_dma_ctrl_block_info));
	for (uint32_t i = 0; i < TRACESWO_RX_DMA_NUMBER_OF_BUFFERS; i++) {
		rx_dma_ctrl_block_info[i].address = (rx_buf[i]);
	}

	uint32_t wait_time = TRACESWO_TASK_NOTIFY_WAIT_PERIOD;

	while (1) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(wait_time)) == pdPASS) {
			if (notificationValue & (USB_SERIAL_DATA_RX_AVAILABLE | USB_SERIAL_DATA_RX_FLUSH)) {
				if (notificationValue & USB_SERIAL_DATA_RX_AVAILABLE) {
					assert(rx_use_dma == false);
					rx_int_process();
				}

				if (notificationValue & USB_SERIAL_DATA_RX_FLUSH) {
					if (rx_use_dma == false) {
						rx_int_finish();
					} else {
						rx_dma_process_buffers();
					}
				}
			}

			if ((notificationValue & USB_SERIAL_DATA_RX_TIMEOUT) && (rx_ongoing != false)) {
				rx_dma_finish_receiving();
			}
		}

		traceswo_update_led();
	}
}

static void traceswo_rx_uart_handler(void)
{
	traceISR_ENTER();

	const uint32_t uart_int_status = rp_uart_get_int_status(TRACESWO_UART);
	assert(uart_int_status != 0);

	uint32_t notify_bits = 0;

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (rx_use_dma == false) {
		if (uart_int_status & RP_UART_INT_RX_BITS) {
			for (uint32_t i = 0; i < (TRACESWO_UART_RX_INT_FIFO_LEVEL - 1); i++) {
				if (rp_uart_is_rx_fifo_empty(TRACESWO_UART)) {
					break;
				}

				((uint8_t *)rx_buf)[rx_int_buf_pos++] = rp_uart_read(TRACESWO_UART);
				if (rx_int_buf_pos >= sizeof(rx_buf)) {
					rx_int_buf_pos = 0;
				}
			}

			rp_uart_clear_rx_irq_flag(TRACESWO_UART);
			rp_uart_set_rx_irq_enabled(TRACESWO_UART, false);
			notify_bits |= USB_SERIAL_DATA_RX_AVAILABLE;
		}

		if (uart_int_status & RP_UART_INT_RX_TIMEOUT_BITS) {
			rp_uart_clear_rx_timeout_irq_flag(TRACESWO_UART);
			rp_uart_set_rx_timeout_irq_enabled(TRACESWO_UART, false);
			notify_bits |= USB_SERIAL_DATA_RX_FLUSH;
		}
	} else {
		higher_priority_task_woken = rx_dma_start_receiving();
		rp_uart_clear_rx_and_rx_timeout_irq_flags(TRACESWO_UART);

		traceswo_update_led();

		portYIELD_FROM_ISR(higher_priority_task_woken);

		return;
	}

	xTaskNotifyFromISR(traceswo_task, notify_bits, eSetBits, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

BaseType_t traceswo_rx_dma_handler(void)
{
	assert(rx_dma_channel != -1);

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (dma_channel_get_irq0_status(rx_dma_channel)) {
		dma_channel_acknowledge_irq0(rx_dma_channel);

		rx_dma_buffer_full_mask |= (1UL << rx_dma_current_buffer);

		if (++rx_dma_current_buffer >= TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) {
			rx_dma_current_buffer = 0;
		}

		xTaskNotifyFromISR(traceswo_task, USB_SERIAL_DATA_RX_FLUSH, eSetBits, &higher_priority_task_woken);
	}

	return higher_priority_task_woken;
}

static void traceswo_rx_pio_handler(void)
{
	static volatile uint32_t test_count = 0;
	test_count++;
	traceISR_ENTER();

	const uint32_t pio_int_status = tap_pio_common_get_irq0_status(TRACESWO_PIO);
	assert(pio_int_status != 0);

	uint32_t notify_bits = 0;

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (rx_use_dma == false) {
		if (pio_int_status & pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM)) {

			while (1)
			{
				if (pio_sm_get_rx_fifo_level(TRACESWO_PIO, TRACESWO_PIO_SM) == 0)
				{
					break;
				}

				((uint8_t *)rx_buf)[rx_int_buf_pos++] = pio_sm_get(TRACESWO_PIO, TRACESWO_PIO_SM);
				if (rx_int_buf_pos >= sizeof(rx_buf)) {
					rx_int_buf_pos = 0;
				}
			}

			pio_set_irq0_source_enabled(TRACESWO_PIO, pio_get_rx_fifo_not_empty_interrupt_source(TRACESWO_PIO_SM), false);
			notify_bits |= USB_SERIAL_DATA_RX_AVAILABLE;
		}
	} else {
		higher_priority_task_woken = rx_dma_start_receiving();

		traceswo_update_led();

		portYIELD_FROM_ISR(higher_priority_task_woken);

		return;
	}

	xTaskNotifyFromISR(traceswo_task, notify_bits, eSetBits, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}