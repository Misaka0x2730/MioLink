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

#define TRACESWO_UART_RX_INT_FIFO_LEVEL (16)

#define TRACESWO_UART_DMA_RX_TOTAL_BUFFERS_SIZE (16 * 1024)
#define TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS  (32)
#define TRACESWO_UART_DMA_RX_BUFFER_SIZE \
	(TRACESWO_UART_DMA_RX_TOTAL_BUFFERS_SIZE / TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS)

#define TRACESWO_UART_DMA_RX_DROP_BUFFER_THRESHOLD (TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS / 3)

#if (TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define TRACESWO_UART_DMA_RX_BAUDRATE_THRESHOLD (38400)
#define TRACESWO_UART_DMA_RX_MIN_TIMEOUT        (50)
#define TRACESWO_UART_DMA_RX_MAX_TIMEOUT        (200)

#define TRACESWO_TASK_NOTIFY_WAIT_PERIOD (portMAX_DELAY)

#define TRACESWO_VENDOR_INTERFACE (0)

#define TRACESWO_DECODE_THRESHOLD (64)

#define TRACESWO_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define TRACESWO_TASK_STACK_SIZE    (512)

static uint8_t uart_rx_buf[TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS][TRACESWO_UART_DMA_RX_BUFFER_SIZE] = {0};
static uint32_t uart_rx_int_buf_pos = 0;
static uint32_t uart_rx_dma_buffer_full_mask = 0;
static uint32_t uart_rx_dma_current_buffer = 0;
static bool uart_rx_use_dma = false;
static xTimerHandle uart_rx_dma_timeout_timer;
static int uart_dma_rx_channel = -1;
static int uart_dma_rx_ctrl_channel = -1;
static uint32_t uart_rx_next_buffer_to_send = 0;
static bool uart_rx_ongoing = false;

/* Current SWO decoding mode being used */
swo_coding_e swo_current_mode = swo_none;

static struct {
	uint8_t *address;
} uart_dma_rx_ctrl_block_info[TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS + 1]
	__attribute__((aligned(TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS * sizeof(uint32_t))));

static bool traceswo_decoding = false;

static TaskHandle_t traceswo_task;

static void uart_rx_isr(void);
static void uart_dma_handler(void);

void traceswo_update_led(void)
{
	platform_set_serial_state(uart_rx_ongoing);
}

static bool usb_trace_send_to_usb(uint8_t *data, const size_t len, const bool flush, const bool allow_drop_buffer)
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

static void uart_rx_dma_init(const uint32_t baudrate)
{
	rp_uart_set_dma_req_enabled(TRACESWO_UART, false, false);
	rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, false, false);
	rp_uart_set_int_fifo_levels(TRACESWO_UART, 0, 0);

	dma_channel_set_irq0_enabled(uart_dma_rx_ctrl_channel, false);
	dma_channel_abort(uart_dma_rx_ctrl_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_ctrl_channel);

	dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
	dma_channel_abort(uart_dma_rx_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_channel);

	dma_channel_config rx_ctrl_config = dma_channel_get_default_config(uart_dma_rx_ctrl_channel);
	channel_config_set_transfer_data_size(&rx_ctrl_config, DMA_SIZE_32);
	channel_config_set_read_increment(&rx_ctrl_config, true);
	channel_config_set_write_increment(&rx_ctrl_config, false);
	channel_config_set_high_priority(&rx_ctrl_config, true);
	channel_config_set_ring(&rx_ctrl_config, false, 7);

	dma_channel_configure(uart_dma_rx_ctrl_channel, &rx_ctrl_config,
		rp_dma_get_al2_write_addr_trig(uart_dma_rx_channel), uart_dma_rx_ctrl_block_info, 1, false);

	dma_channel_config rx_config = dma_channel_get_default_config(uart_dma_rx_channel);
	channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
	channel_config_set_read_increment(&rx_config, false);
	channel_config_set_write_increment(&rx_config, true);
	channel_config_set_dreq(&rx_config, uart_get_dreq(TRACESWO_UART, false));
	channel_config_set_high_priority(&rx_config, true);
	channel_config_set_chain_to(&rx_config, uart_dma_rx_ctrl_channel);

	dma_channel_configure(uart_dma_rx_channel, &rx_config, uart_rx_buf, rp_uart_get_dr_address(TRACESWO_UART),
		TRACESWO_UART_DMA_RX_BUFFER_SIZE, false);

	uart_rx_use_dma = true;

	rp_dma_set_channel_enabled(uart_dma_rx_channel, false, false);
	dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

	dma_channel_set_read_addr(uart_dma_rx_ctrl_channel, (void *)uart_dma_rx_ctrl_block_info, true);

	/* Calculate timer period - time to fill 2 rx buffers */
	uint32_t timer_period = (TRACESWO_UART_DMA_RX_BUFFER_SIZE * 2 * 1000); /* 1000 - because we need milliseconds */
	timer_period /= (baudrate / 10); /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
	if (timer_period < TRACESWO_UART_DMA_RX_MIN_TIMEOUT) {
		timer_period = TRACESWO_UART_DMA_RX_MIN_TIMEOUT;
	} else if (timer_period > TRACESWO_UART_DMA_RX_MAX_TIMEOUT) {
		timer_period = TRACESWO_UART_DMA_RX_MAX_TIMEOUT;
	}

	xTimerChangePeriod(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

	rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);
}

static BaseType_t uart_rx_dma_start_receiving(void)
{
	assert(uart_rx_ongoing == false);

	uart_rx_ongoing = true;

	rp_dma_set_channel_enabled(uart_dma_rx_channel, true, false);

	rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, false, false);
	rp_uart_clear_rx_and_rx_timeout_irq_flags(TRACESWO_UART);

	dma_channel_acknowledge_irq0(uart_dma_rx_channel);
	dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

	rp_uart_set_dma_req_enabled(TRACESWO_UART, true, false);

	BaseType_t higher_priority_task_woken = pdFALSE;

	xTimerResetFromISR(uart_rx_dma_timeout_timer, &higher_priority_task_woken);
	return higher_priority_task_woken;
}

static void uart_rx_dma_process_buffers(void)
{
	xTimerReset(uart_rx_dma_timeout_timer, 0);

	while (1) {
		const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1u << uart_rx_next_buffer_to_send);
		const bool allow_drop_buffer = (__builtin_popcount(buffer_state) >= TRACESWO_UART_DMA_RX_DROP_BUFFER_THRESHOLD);
		const uint32_t data_len = sizeof(uart_rx_buf[uart_rx_next_buffer_to_send]);
		if (buffer_state & buffer_bit) {
			if (traceswo_decoding) {
				if (traceswo_decode(uart_rx_buf[uart_rx_next_buffer_to_send], data_len, false, allow_drop_buffer)) {
					Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~buffer_bit);
					if (++uart_rx_next_buffer_to_send >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
						uart_rx_next_buffer_to_send = 0;
					}
				} else {
					xTimerReset(uart_rx_dma_timeout_timer, 0);
					vTaskDelay(pdMS_TO_TICKS(1));
					continue;
				}
			} else if (usb_trace_send_to_usb(
						   uart_rx_buf[uart_rx_next_buffer_to_send], data_len, false, allow_drop_buffer)) {
				Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~buffer_bit);
				if (++uart_rx_next_buffer_to_send >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
					uart_rx_next_buffer_to_send = 0;
				}
			}
		} else {
			break;
		}
	}
}

static void uart_rx_dma_timeout_callback(TimerHandle_t xTimer)
{
	(void)(xTimer);
	xTaskNotify(traceswo_task, USB_SERIAL_DATA_UART_RX_TIMEOUT, eSetBits);
}

static bool uart_rx_dma_finish_receiving(void)
{
	assert(uart_rx_ongoing != false);

	rp_uart_set_dma_req_enabled(TRACESWO_UART, false, false);

	dma_channel_set_irq0_enabled(uart_dma_rx_ctrl_channel, false);
	dma_channel_abort(uart_dma_rx_ctrl_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_ctrl_channel);

	const uint32_t current_buffer = uart_rx_dma_current_buffer;

	if (++uart_rx_dma_current_buffer >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
		uart_rx_dma_current_buffer = 0;
	}

	xTimerStop(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(0));

	uart_rx_ongoing = false;

	const uint32_t remaining = rp_dma_get_trans_count(uart_dma_rx_channel);
	const uint32_t data_in_buffer = sizeof(uart_rx_buf[0]) - remaining;

	dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
	dma_channel_abort(uart_dma_rx_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_channel);

	dma_channel_set_read_addr(
		uart_dma_rx_ctrl_channel, (void *)(uart_dma_rx_ctrl_block_info + uart_rx_dma_current_buffer), true);
	rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);

	while (1) {
		const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1u << uart_rx_next_buffer_to_send);
		if (buffer_state & buffer_bit) {
			if (traceswo_decoding) {
				traceswo_decode(uart_rx_buf[uart_rx_next_buffer_to_send],
					sizeof(uart_rx_buf[uart_rx_next_buffer_to_send]), false, true);
			} else {
				usb_trace_send_to_usb(uart_rx_buf[uart_rx_next_buffer_to_send],
					sizeof(uart_rx_buf[uart_rx_next_buffer_to_send]), false, true);
			}

			Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~buffer_bit);
			if (++uart_rx_next_buffer_to_send >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
				uart_rx_next_buffer_to_send = 0;
			}
		} else {
			break;
		}
	}

	if ((current_buffer + 1) >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
		uart_rx_next_buffer_to_send = 0;
	} else {
		uart_rx_next_buffer_to_send = current_buffer + 1;
	}

	if (traceswo_decoding) {
		traceswo_decode(uart_rx_buf[current_buffer], data_in_buffer, true, true);
	} else {
		usb_trace_send_to_usb(uart_rx_buf[current_buffer], data_in_buffer, true, true);
	}

	return false;
}

static void uart_rx_int_init(void)
{
	uart_rx_use_dma = false;

	/* Set RX FIFO level to 1/2 */
	rp_uart_set_int_fifo_levels(TRACESWO_UART, 2, 0);
	rp_uart_set_rx_and_timeout_irq_enabled(TRACESWO_UART, true, true);
}

static void uart_rx_int_process(void)
{
	if (uart_rx_int_buf_pos > 0) {
		if ((traceswo_decoding) && (uart_rx_int_buf_pos >= TRACESWO_DECODE_THRESHOLD)) {
			/* write decoded swo packets to the uart port */
			traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos, false, true);

			uart_rx_int_buf_pos = 0;
		} else if (usb_trace_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, false, true) != false) {
			uart_rx_int_buf_pos = 0;
		}

		uart_rx_ongoing = true;
	}

	rp_uart_set_rx_irq_enabled(TRACESWO_UART, true);
}

static void uart_rx_int_finish(void)
{
	if (uart_rx_int_buf_pos > 0) {
		usb_trace_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, false, true);
		uart_rx_int_buf_pos = 0;
	}

	while (rp_uart_is_rx_fifo_empty(TRACESWO_UART) == false) {
		((uint8_t *)uart_rx_buf)[uart_rx_int_buf_pos++] = rp_uart_read(TRACESWO_UART);

		if (uart_rx_int_buf_pos >= sizeof(uart_rx_buf)) {
			uart_rx_int_buf_pos = 0;
		}
	}

	if (uart_rx_int_buf_pos > 0) {
		if (traceswo_decoding) {
			/* write decoded swo packets to the uart port */
			traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos, true, true);
		} else {
			usb_trace_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, true, true);
		}

		uart_rx_int_buf_pos = 0;
	}

	uart_rx_ongoing = false;
	rp_uart_set_rx_timeout_irq_enabled(TRACESWO_UART, true);
}

void swo_init(swo_coding_e swo_mode, uint32_t baudrate, uint32_t itm_stream_bitmask)
{
	usb_serial_uart_release(TRACESWO_UART);

	portENTER_CRITICAL();

	swo_deinit(false);

	const platform_target_pins_t *target_pins = platform_get_target_pins();

	/* Re-initialize UART */
	gpio_set_function(target_pins->tdo, GPIO_FUNC_UART);
	gpio_set_function(target_pins->tdi, GPIO_FUNC_SIO);

	irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_UART_IRQ);
	assert(current_handler == NULL);

	irq_set_exclusive_handler(TRACESWO_UART_IRQ, uart_rx_isr);

	if (baudrate == 0) {
		baudrate = SWO_DEFAULT_BAUD;
	}

	uart_init(TRACESWO_UART, baudrate);

	if (baudrate >= TRACESWO_UART_DMA_RX_BAUDRATE_THRESHOLD) {
		uart_rx_dma_init(baudrate);
	} else {
		uart_rx_int_init();
	}

	memset(uart_rx_buf, 0xAA, sizeof(uart_rx_buf));

	irq_set_enabled(TRACESWO_UART_IRQ, true);

	traceswo_setmask(itm_stream_bitmask);
	traceswo_decoding = itm_stream_bitmask != 0;

	swo_current_mode = swo_mode;

	portEXIT_CRITICAL();

	gdb_outf("Baudrate: %" PRIu32 " ", swo_uart_get_baudrate());
}

void swo_deinit(bool deallocate)
{
	(void)(deallocate);

	if (swo_current_mode != swo_none) {
		portENTER_CRITICAL();

		dma_channel_set_irq0_enabled(uart_dma_rx_ctrl_channel, false);
		dma_channel_abort(uart_dma_rx_ctrl_channel);
		dma_channel_acknowledge_irq0(uart_dma_rx_ctrl_channel);

		dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
		dma_channel_abort(uart_dma_rx_channel);
		dma_channel_acknowledge_irq0(uart_dma_rx_channel);

		irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_UART_IRQ);
		irq_set_enabled(TRACESWO_UART_IRQ, false);

		if (current_handler != NULL) {
			irq_remove_handler(TRACESWO_UART_IRQ, current_handler);
		}

		uart_deinit(TRACESWO_UART);

		uart_rx_int_buf_pos = 0;
		uart_rx_ongoing = false;

		portEXIT_CRITICAL();
	}

	swo_current_mode = swo_none;
}

bool traceswo_uart_is_used(uart_inst_t *uart_instance)
{
	return ((swo_current_mode != swo_none) && (uart_instance == TRACESWO_UART));
}

uint32_t swo_uart_get_baudrate(void)
{
	return rp_uart_get_baudrate(TRACESWO_UART);
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

	uart_rx_dma_timeout_timer = xTimerCreate(
		"TRACE_UART_RX", pdMS_TO_TICKS(TRACESWO_UART_DMA_RX_MAX_TIMEOUT), pdFALSE, NULL, uart_rx_dma_timeout_callback);

	uart_dma_rx_channel = dma_claim_unused_channel(true);
	uart_dma_rx_ctrl_channel = dma_claim_unused_channel(true);

	memset(&uart_dma_rx_ctrl_block_info, 0, sizeof(uart_dma_rx_ctrl_block_info));
	for (uint32_t i = 0; i < TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS; i++) {
		uart_dma_rx_ctrl_block_info[i].address = (uart_rx_buf[i]);
	}

	uint32_t wait_time = TRACESWO_TASK_NOTIFY_WAIT_PERIOD;

	while (1) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(wait_time)) == pdPASS) {
			if (notificationValue & (USB_SERIAL_DATA_UART_RX_AVAILABLE | USB_SERIAL_DATA_UART_RX_FLUSH)) {
				if (notificationValue & USB_SERIAL_DATA_UART_RX_AVAILABLE) {
					assert(uart_rx_use_dma == false);
					uart_rx_int_process();
				}

				if (notificationValue & USB_SERIAL_DATA_UART_RX_FLUSH) {
					if (uart_rx_use_dma == false) {
						uart_rx_int_finish();
					} else {
						uart_rx_dma_process_buffers();
					}
				}
			}

			if ((notificationValue & USB_SERIAL_DATA_UART_RX_TIMEOUT) && (uart_rx_ongoing != false)) {
				uart_rx_dma_finish_receiving();
			}
		}

		traceswo_update_led();
	}
}

static void uart_rx_isr(void)
{
	traceISR_ENTER();

	const uint32_t uart_int_status = rp_uart_get_int_status(TRACESWO_UART);
	assert(uart_int_status != 0);

	uint32_t notify_bits = 0;

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (uart_rx_use_dma == false) {
		if (uart_int_status & RP_UART_INT_RX_BITS) {
			for (uint32_t i = 0; i < (TRACESWO_UART_RX_INT_FIFO_LEVEL - 1); i++) {
				if (rp_uart_is_rx_fifo_empty(TRACESWO_UART)) {
					break;
				}

				((uint8_t *)uart_rx_buf)[uart_rx_int_buf_pos++] = rp_uart_read(TRACESWO_UART);
				if (uart_rx_int_buf_pos >= sizeof(uart_rx_buf)) {
					uart_rx_int_buf_pos = 0;
				}
			}

			rp_uart_clear_rx_irq_flag(TRACESWO_UART);
			rp_uart_set_rx_irq_enabled(TRACESWO_UART, false);
			notify_bits |= USB_SERIAL_DATA_UART_RX_AVAILABLE;
		}

		if (uart_int_status & RP_UART_INT_RX_TIMEOUT_BITS) {
			rp_uart_clear_rx_timeout_irq_flag(TRACESWO_UART);
			rp_uart_set_rx_timeout_irq_enabled(TRACESWO_UART, false);
			notify_bits |= USB_SERIAL_DATA_UART_RX_FLUSH;
		}
	} else {
		higher_priority_task_woken = uart_rx_dma_start_receiving();
		rp_uart_clear_rx_and_rx_timeout_irq_flags(TRACESWO_UART);

		traceswo_update_led();

		portYIELD_FROM_ISR(higher_priority_task_woken);

		return;
	}

	xTaskNotifyFromISR(traceswo_task, notify_bits, eSetBits, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

BaseType_t traceswo_uart_dma_handler(void)
{
	assert(uart_dma_rx_channel != -1);

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (dma_channel_get_irq0_status(uart_dma_rx_channel)) {
		dma_channel_acknowledge_irq0(uart_dma_rx_channel);

		uart_rx_dma_buffer_full_mask |= (1UL << uart_rx_dma_current_buffer);

		if (++uart_rx_dma_current_buffer >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS) {
			uart_rx_dma_current_buffer = 0;
		}

		xTaskNotifyFromISR(traceswo_task, USB_SERIAL_DATA_UART_RX_FLUSH, eSetBits, &higher_priority_task_woken);
	}

	return higher_priority_task_woken;
}