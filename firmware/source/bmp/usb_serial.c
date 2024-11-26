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

#if ENABLE_DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef PLATFORM_HAS_TRACESWO
#include "swo.h"
#endif

#ifdef ENABLE_RTT
#include "rtt.h"
#endif

#include "usb_serial.h"

#define USB_SERIAL_UART_RX_INT_FIFO_LEVEL (16)

#define USB_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE (16 * 1024)
#define USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS  (32)
#define USB_SERIAL_UART_DMA_RX_BUFFER_SIZE \
	(USB_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE / USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS)

#define USB_SERIAL_UART_DMA_RX_DROP_BUFFER_THRESHOLD (USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS / 3)

#if (USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define USB_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD (38400)
#define USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT        (50)
#define USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT        (200)

#define USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD (portMAX_DELAY)

#define USB_SERIAL_UART_DMA_TX_BUFFER_SIZE           (256)
#define USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD (2)

#define USB_SERIAL_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define USB_SERIAL_TASK_STACK_SIZE    (512)

static uint8_t uart_rx_buf[USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS][USB_SERIAL_UART_DMA_RX_BUFFER_SIZE] = {0};
static uint32_t uart_rx_int_buf_pos = 0;
static uint32_t uart_rx_dma_buffer_full_mask = 0;
static uint32_t uart_rx_dma_current_buffer = 0;
static bool uart_rx_use_dma = false;
static xTimerHandle uart_rx_dma_timeout_timer;
static int uart_dma_rx_channel = -1;
static int uart_dma_rx_ctrl_channel = -1;
static uint32_t uart_rx_next_buffer_to_send = 0;
static bool uart_rx_ongoing = false;

static struct {
	uint8_t *address;
} uart_dma_rx_ctrl_block_info[USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS + 1]
	__attribute__((aligned(USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS * sizeof(uint32_t))));

static int uart_dma_tx_channel = -1;
uint8_t uart_tx_dma_buf[USB_SERIAL_UART_DMA_TX_BUFFER_SIZE] = {0};
bool uart_tx_dma_finished = false;
static bool uart_tx_ongoing = false;

TaskHandle_t usb_uart_task;

/* Sets externally */
bool use_uart_on_tdi_tdo = false;

static uart_inst_t *current_uart = USB_SERIAL_UART_MAIN;

static void uart_rx_isr(void);
static void uart_dma_handler(void);

bool usb_serial_get_dtr(void)
{
	return (tud_cdc_n_get_line_state(USB_SERIAL_TARGET) & 0x01) != 0;
}

void usb_serial_update_led(void)
{
	if (tud_cdc_n_connected(USB_SERIAL_TARGET) == false) {
		platform_set_serial_state(false);
	} else {
		platform_set_serial_state(uart_rx_ongoing || uart_tx_ongoing);
	}
}

uint16_t usb_serial_get_available(void)
{
	return tud_cdc_n_write_available(USB_SERIAL_TARGET);
}

uint32_t usb_serial_read(uint8_t *data, const uint32_t buffer_size)
{
	return tud_cdc_n_read(USB_SERIAL_TARGET, data, buffer_size);
}

bool usb_serial_send_to_usb(uint8_t *data, const size_t len, bool flush, const bool allow_drop_buffer)
{
	bool result = false;
	const uint32_t write_available = tud_cdc_n_write_available(USB_SERIAL_TARGET);

	if (usb_serial_get_dtr() == false) {
		return true;
	}

	if (write_available < len) {
		if (allow_drop_buffer) {
			tud_cdc_n_write(USB_SERIAL_TARGET, data, write_available);
			result = true;
		}
	} else if (len > 0) {
		result = (tud_cdc_n_write(USB_SERIAL_TARGET, data, len) == len);
	} else {
		result = true;
		flush = true;
	}

	if (result && flush) {
		tud_cdc_n_write_flush(USB_SERIAL_TARGET);
	}

	return result;
}

static void uart_rx_dma_init(const uint32_t baudrate)
{
	rp_uart_set_dma_req_enabled(current_uart, false, true);
	rp_uart_set_rx_and_timeout_irq_enabled(current_uart, false, false);
	rp_uart_set_int_fifo_levels(current_uart, 0, 0);

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
	channel_config_set_dreq(&rx_config, uart_get_dreq(current_uart, false));
	channel_config_set_high_priority(&rx_config, true);
	channel_config_set_chain_to(&rx_config, uart_dma_rx_ctrl_channel);

	dma_channel_configure(uart_dma_rx_channel, &rx_config, uart_rx_buf, rp_uart_get_dr_address(current_uart),
		USB_SERIAL_UART_DMA_RX_BUFFER_SIZE, false);

	uart_rx_use_dma = true;

	rp_dma_set_channel_enabled(uart_dma_rx_channel, false, false);
	dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

	dma_channel_set_read_addr(uart_dma_rx_ctrl_channel, (void *)uart_dma_rx_ctrl_block_info, true);

	/* Calculate timer period - time to fill 2 rx buffers */
	uint32_t timer_period = (USB_SERIAL_UART_DMA_RX_BUFFER_SIZE * 2 * 1000); /* 1000 - because we need milliseconds */
	timer_period /= (baudrate / 10); /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
	if (timer_period < USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT) {
		timer_period = USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT;
	} else if (timer_period > USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT) {
		timer_period = USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT;
	}

	xTimerChangePeriod(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

	rp_uart_set_rx_and_timeout_irq_enabled(current_uart, true, true);
}

static BaseType_t uart_rx_dma_start_receiving(void)
{
	assert(uart_rx_ongoing == false);

	uart_rx_ongoing = true;

	rp_dma_set_channel_enabled(uart_dma_rx_channel, true, false);

	rp_uart_set_rx_and_timeout_irq_enabled(current_uart, false, false);
	rp_uart_clear_rx_and_rx_timeout_irq_flags(current_uart);

	dma_channel_acknowledge_irq0(uart_dma_rx_channel);
	dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

	rp_uart_set_dma_req_enabled(current_uart, true, true);

	BaseType_t higher_priority_task_woken = pdFALSE;

	xTimerResetFromISR(uart_rx_dma_timeout_timer, &higher_priority_task_woken);
	return higher_priority_task_woken;
}

static void uart_rx_dma_process_buffers(void)
{
	xTimerReset(uart_rx_dma_timeout_timer, 0);

	while (1) {
		const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1UL << uart_rx_next_buffer_to_send);
		const bool allow_drop_buffer =
			(__builtin_popcount(buffer_state) >= USB_SERIAL_UART_DMA_RX_DROP_BUFFER_THRESHOLD);
		const uint32_t data_len = sizeof(uart_rx_buf[uart_rx_next_buffer_to_send]);
		if (buffer_state & buffer_bit) {
			if (usb_serial_send_to_usb(uart_rx_buf[uart_rx_next_buffer_to_send], data_len, false, allow_drop_buffer)) {
				Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~buffer_bit);
				if (++uart_rx_next_buffer_to_send >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) {
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
	xTaskNotify(usb_uart_task, USB_SERIAL_DATA_UART_RX_TIMEOUT, eSetBits);
}

static bool uart_rx_dma_finish_receiving(void)
{
	assert(uart_rx_ongoing != false);

	rp_uart_set_dma_req_enabled(current_uart, false, true);

	dma_channel_set_irq0_enabled(uart_dma_rx_ctrl_channel, false);
	dma_channel_abort(uart_dma_rx_ctrl_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_ctrl_channel);

	const uint32_t current_buffer = uart_rx_dma_current_buffer;

	if (++uart_rx_dma_current_buffer >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) {
		uart_rx_dma_current_buffer = 0;
	}

	uart_rx_ongoing = false;

	xTimerStop(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(0));

	const uint32_t remaining = rp_dma_get_trans_count(uart_dma_rx_channel);
	const uint32_t data_in_buffer = sizeof(uart_rx_buf[0]) - remaining;

	dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
	rp_dma_set_chain_to(uart_dma_rx_channel, uart_dma_rx_channel);
	dma_channel_abort(uart_dma_rx_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_channel);
	rp_dma_set_chain_to(uart_dma_rx_channel, uart_dma_rx_ctrl_channel);

	dma_channel_set_read_addr(
		uart_dma_rx_ctrl_channel, (void *)(uart_dma_rx_ctrl_block_info + uart_rx_dma_current_buffer), true);
	rp_uart_set_rx_and_timeout_irq_enabled(current_uart, true, true);

	while (1) {
		const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1UL << uart_rx_next_buffer_to_send);
		if (buffer_state & buffer_bit) {
			usb_serial_send_to_usb(uart_rx_buf[uart_rx_next_buffer_to_send],
				sizeof(uart_rx_buf[uart_rx_next_buffer_to_send]), false, true);

			Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~buffer_bit);
			if (++uart_rx_next_buffer_to_send >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) {
				uart_rx_next_buffer_to_send = 0;
			}
		} else {
			break;
		}
	}

	if ((current_buffer + 1) >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) {
		uart_rx_next_buffer_to_send = 0;
	} else {
		uart_rx_next_buffer_to_send = current_buffer + 1;
	}

	usb_serial_send_to_usb(uart_rx_buf[current_buffer], data_in_buffer, true, true);

	return false;
}

static void uart_rx_int_init(void)
{
	uart_rx_use_dma = false;

	/* Set RX FIFO level to 1/2 */
	rp_uart_set_int_fifo_levels(current_uart, 2, 0);
	rp_uart_set_rx_and_timeout_irq_enabled(current_uart, true, true);
}

static void uart_rx_int_process(void)
{
	if (uart_rx_int_buf_pos > 0) {
		if (usb_serial_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, false, true) != false) {
			uart_rx_int_buf_pos = 0;
		}

		uart_rx_ongoing = true;
	}

	rp_uart_set_rx_irq_enabled(current_uart, true);
}

static void uart_rx_int_finish(void)
{
	if (uart_rx_int_buf_pos > 0) {
		usb_serial_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, false, true);
		uart_rx_int_buf_pos = 0;
	}

	while (rp_uart_is_rx_fifo_empty(current_uart) == false) {
		((uint8_t *)uart_rx_buf)[uart_rx_int_buf_pos++] = rp_uart_read(current_uart);

		if (uart_rx_int_buf_pos >= sizeof(uart_rx_buf)) {
			uart_rx_int_buf_pos = 0;
		}
	}

	if (uart_rx_int_buf_pos > 0) {
		usb_serial_send_to_usb((uint8_t *)(uart_rx_buf), uart_rx_int_buf_pos, true, true);

		uart_rx_int_buf_pos = 0;
	}

	uart_rx_ongoing = false;
	rp_uart_set_rx_timeout_irq_enabled(current_uart, true);
}

static void uart_tx_dma_send(void)
{
	if (tud_cdc_n_available(USB_SERIAL_TARGET)) {
		const uint32_t read_count = tud_cdc_n_read(USB_SERIAL_TARGET, uart_tx_dma_buf, sizeof(uart_tx_dma_buf));
		if (read_count != 0) {
			dma_channel_acknowledge_irq0(uart_dma_tx_channel);
			dma_channel_set_irq0_enabled(uart_dma_tx_channel, false);

			dma_channel_set_read_addr(uart_dma_tx_channel, uart_tx_dma_buf, false);
			dma_channel_set_write_addr(uart_dma_tx_channel, rp_uart_get_dr_address(current_uart), false);
			dma_channel_set_trans_count(uart_dma_tx_channel, read_count, true);

			uart_tx_ongoing = true;

			dma_channel_set_irq0_enabled(uart_dma_tx_channel, true);
		} else if (uart_tx_ongoing) {
			uart_tx_dma_finished = true;
		}
	} else if (uart_tx_ongoing) {
		uart_tx_dma_finished = true;
	}
}

static bool uart_tx_dma_check_uart_finished(void)
{
	return !rp_uart_is_transmitting(current_uart);
}

static void uart_update_config(cdc_line_coding_t *line_coding)
{
	uint32_t stop_bits = 2;
	switch (line_coding->stop_bits) {
	case 0:
	case 1:
		stop_bits = 1;
		break;
	case 2:
	default:
		break;
	}

	uart_parity_t parity = UART_PARITY_NONE;
	switch (line_coding->parity) {
	case 0:
	default:
		break;
	case 1:
		parity = UART_PARITY_ODD;
		break;
	case 2:
		parity = UART_PARITY_EVEN;
		break;
	}

	uint8_t data_bits = 8;

	if (line_coding->data_bits <= 8) {
		data_bits = line_coding->data_bits;
	}

	portENTER_CRITICAL();
	xTimerStop(uart_rx_dma_timeout_timer, 0);

	dma_channel_set_irq0_enabled(uart_dma_rx_ctrl_channel, false);
	dma_channel_abort(uart_dma_rx_ctrl_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_ctrl_channel);

	dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
	dma_channel_abort(uart_dma_rx_channel);
	dma_channel_acknowledge_irq0(uart_dma_rx_channel);

	dma_channel_set_irq0_enabled(uart_dma_tx_channel, false);
	dma_channel_abort(uart_dma_tx_channel);
	dma_channel_acknowledge_irq0(uart_dma_tx_channel);

	if ((use_uart_on_tdi_tdo != false) && (current_uart == USB_SERIAL_UART_MAIN)) {
#ifdef PLATFORM_HAS_TRACESWO
		if (traceswo_uart_is_used(USB_SERIAL_UART_TDI_TDO)) {
			return;
		}
#endif

		irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);
		irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, false);
		irq_remove_handler(USB_SERIAL_UART_MAIN_IRQ, current_handler);

		current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ);
		irq_set_enabled(USB_SERIAL_UART_TDI_TDO_IRQ, false);
		irq_remove_handler(USB_SERIAL_UART_TDI_TDO_IRQ, current_handler);

		uart_deinit(USB_SERIAL_UART_MAIN);
		current_uart = USB_SERIAL_UART_TDI_TDO;
	} else if ((use_uart_on_tdi_tdo == false) && (current_uart == USB_SERIAL_UART_TDI_TDO)) {
#ifdef PLATFORM_HAS_TRACESWO
		if (traceswo_uart_is_used(USB_SERIAL_UART_MAIN)) {
			return;
		}
#endif

		irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);
		irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, false);
		irq_remove_handler(USB_SERIAL_UART_MAIN_IRQ, current_handler);

		current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ);
		irq_set_enabled(USB_SERIAL_UART_TDI_TDO_IRQ, false);
		irq_remove_handler(USB_SERIAL_UART_TDI_TDO_IRQ, current_handler);

		uart_deinit(USB_SERIAL_UART_TDI_TDO);
		current_uart = USB_SERIAL_UART_MAIN;
	}

	const platform_target_pins_t *target_pins = platform_get_target_pins();

	if (current_uart == USB_SERIAL_UART_MAIN) {
		gpio_set_function(target_pins->uart_tx, GPIO_FUNC_UART);
		gpio_set_function(target_pins->uart_rx, GPIO_FUNC_UART);

		irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);

		if (current_handler != NULL) {
			irq_remove_handler(USB_SERIAL_UART_MAIN_IRQ, current_handler);
		}
		irq_set_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ, uart_rx_isr);
		irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, true);
	} else {
		gpio_set_function(target_pins->tdo, GPIO_FUNC_UART);
		gpio_set_function(target_pins->tdi, GPIO_FUNC_UART);

		irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ);
		if (current_handler != NULL) {
			irq_remove_handler(USB_SERIAL_UART_TDI_TDO_IRQ, current_handler);
		}
		irq_set_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ, uart_rx_isr);
		irq_set_enabled(USB_SERIAL_UART_TDI_TDO_IRQ, true);
	}

	dma_channel_config tx_config = dma_channel_get_default_config(uart_dma_tx_channel);
	channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
	channel_config_set_read_increment(&tx_config, true);
	channel_config_set_write_increment(&tx_config, false);

	channel_config_set_dreq(&tx_config, uart_get_dreq(current_uart, true));

	dma_channel_configure(uart_dma_tx_channel, &tx_config, rp_uart_get_dr_address(current_uart), uart_tx_dma_buf,
		sizeof(uart_tx_dma_buf), false);

	uart_init(current_uart, line_coding->bit_rate);
	uart_set_format(current_uart, data_bits, stop_bits, parity);

	uart_rx_int_buf_pos = 0;
	uart_rx_ongoing = false;
	uart_tx_ongoing = false;

	if (line_coding->bit_rate >= USB_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD) {
		uart_rx_dma_init(line_coding->bit_rate);
	} else {
		uart_rx_int_init();
	}

	portEXIT_CRITICAL();
}

void usb_serial_uart_release(uart_inst_t *uart_to_release)
{
	if (uart_to_release == current_uart) {
		if (current_uart == USB_SERIAL_UART_TDI_TDO) {
			use_uart_on_tdi_tdo = false;
		} else {
			use_uart_on_tdi_tdo = true;
		}

		cdc_line_coding_t line_coding = {0};
		tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

		uart_update_config(&line_coding);
	}
}

_Noreturn static void target_serial_thread(void *params);

void usb_serial_init(void)
{
#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(target_serial_thread, "target_uart", USB_SERIAL_TASK_STACK_SIZE,
		NULL, PLATFORM_PRIORITY_NORMAL, USB_SERIAL_TASK_CORE_AFFINITY, &usb_uart_task);
#else
	const BaseType_t result = xTaskCreate(target_serial_thread, "target_uart", USB_SERIAL_TASK_STACK_SIZE, NULL,
		PLATFORM_PRIORITY_NORMAL, &usb_uart_task);
#endif

	assert(result == pdPASS);
}

#ifdef ENABLE_RTT
extern void rtt_serial_receive_callback(void);
#endif

_Noreturn static void target_serial_thread(void *params)
{
	(void)params;

	uint32_t notification_value = 0;

	uart_rx_dma_timeout_timer = xTimerCreate("SERIAL_UART_RX", pdMS_TO_TICKS(USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT),
		pdFALSE, NULL, uart_rx_dma_timeout_callback);

	uart_dma_tx_channel = dma_claim_unused_channel(true);
	uart_dma_rx_channel = dma_claim_unused_channel(true);
	uart_dma_rx_ctrl_channel = dma_claim_unused_channel(true);

	memset(&uart_dma_rx_ctrl_block_info, 0, sizeof(uart_dma_rx_ctrl_block_info));
	for (uint32_t i = 0; i < USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS; i++) {
		uart_dma_rx_ctrl_block_info[i].address = (uart_rx_buf[i]);
	}

	irq_set_exclusive_handler(USB_SERIAL_TRACESWO_DMA_IRQ, uart_dma_handler);
	irq_set_enabled(USB_SERIAL_TRACESWO_DMA_IRQ, true);

	uint32_t wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

	while (1) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, pdMS_TO_TICKS(wait_time)) == pdPASS) {
			if (notification_value & USB_SERIAL_LINE_CODING_UPDATE) {
				cdc_line_coding_t line_coding = {0};
				tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

				uart_update_config(&line_coding);
			}

			if (notification_value & (USB_SERIAL_DATA_UART_RX_AVAILABLE | USB_SERIAL_DATA_UART_RX_FLUSH)) {
				if (notification_value & USB_SERIAL_DATA_UART_RX_AVAILABLE) {
					assert(uart_rx_use_dma == false);
					uart_rx_int_process();
				}

				if (notification_value & USB_SERIAL_DATA_UART_RX_FLUSH) {
					if (uart_rx_use_dma == false) {
						uart_rx_int_finish();
					} else {
						uart_rx_dma_process_buffers();
					}
				}
			}

			if (notification_value & USB_SERIAL_DATA_UART_TX_COMPLETE) {
				uart_tx_dma_send();
			}

			if ((notification_value & USB_SERIAL_DATA_UART_RX_TIMEOUT) && (uart_rx_ongoing != false)) {
				uart_rx_dma_finish_receiving();
			}

			if ((notification_value & USB_SERIAL_DATA_RX) && (uart_tx_ongoing == false)) {
#ifdef ENABLE_RTT
				if (rtt_enabled) {
					rtt_serial_receive_callback();
				} else {
					uart_tx_dma_send();
				}
#else
				uart_tx_dma_send();
#endif
			}
		}

		if (uart_tx_dma_finished) {
			if (uart_tx_dma_check_uart_finished()) {
				wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

				uart_tx_ongoing = false;
				uart_tx_dma_finished = false;
			} else {
				wait_time = USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD;
			}
		}

		if (((use_uart_on_tdi_tdo != false) && (current_uart == USB_SERIAL_UART_MAIN)) ||
			((use_uart_on_tdi_tdo == false) && (current_uart == USB_SERIAL_UART_TDI_TDO))) {
			cdc_line_coding_t line_coding = {0};
			tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

			uart_update_config(&line_coding);
		}

		usb_serial_update_led();
	}
}

static void uart_rx_isr(void)
{
	traceISR_ENTER();

	const uint32_t uart_int_status = rp_uart_get_int_status(current_uart);
	assert(uart_int_status != 0);

	uint32_t notify_bits = 0;

	BaseType_t higher_priority_task_woken = pdFALSE;

	if (uart_rx_use_dma == false) {
		if (uart_int_status & RP_UART_INT_RX_BITS) {
			for (uint32_t i = 0; i < (USB_SERIAL_UART_RX_INT_FIFO_LEVEL - 1); i++) {
				if (rp_uart_is_rx_fifo_empty(current_uart)) {
					break;
				}

				((uint8_t *)uart_rx_buf)[uart_rx_int_buf_pos] = rp_uart_read(current_uart);
				if (++uart_rx_int_buf_pos >= sizeof(uart_rx_buf)) {
					uart_rx_int_buf_pos = 0;
				}
			}

			rp_uart_clear_rx_irq_flag(current_uart);
			rp_uart_set_rx_irq_enabled(current_uart, false);
			notify_bits |= USB_SERIAL_DATA_UART_RX_AVAILABLE;
		}

		if (uart_int_status & RP_UART_INT_RX_TIMEOUT_BITS) {
			rp_uart_clear_rx_timeout_irq_flag(current_uart);
			rp_uart_set_rx_timeout_irq_enabled(current_uart, false);
			notify_bits |= USB_SERIAL_DATA_UART_RX_FLUSH;
		}
	} else {
		higher_priority_task_woken = uart_rx_dma_start_receiving();
		rp_uart_clear_rx_and_rx_timeout_irq_flags(current_uart);

		usb_serial_update_led();

		portYIELD_FROM_ISR(higher_priority_task_woken);

		return;
	}

	xTaskNotifyFromISR(usb_uart_task, notify_bits, eSetBits, &higher_priority_task_woken);
	portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void uart_dma_handler(void)
{
	traceISR_ENTER();

	BaseType_t higher_priority_task_woken = pdFALSE;

	assert((uart_dma_rx_channel != -1) && (uart_dma_tx_channel != -1));

	if (dma_channel_get_irq0_status(uart_dma_tx_channel)) {
		dma_channel_set_irq0_enabled(uart_dma_tx_channel, false);
		dma_channel_acknowledge_irq0(uart_dma_tx_channel);

		xTaskNotifyFromISR(usb_uart_task, USB_SERIAL_DATA_UART_TX_COMPLETE, eSetBits, &higher_priority_task_woken);
	}

	if (dma_channel_get_irq0_status(uart_dma_rx_channel)) {
		dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
		uart_rx_dma_buffer_full_mask |= (1UL << uart_rx_dma_current_buffer);

		if (++uart_rx_dma_current_buffer >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) {
			uart_rx_dma_current_buffer = 0;
		}
		-xTaskNotifyFromISR(usb_uart_task, USB_SERIAL_DATA_UART_RX_FLUSH, eSetBits, &higher_priority_task_woken);
	}

#ifdef PLATFORM_HAS_TRACESWO
	const BaseType_t higher_priority_task_woken_trace = traceswo_uart_dma_handler();

	portYIELD_FROM_ISR(higher_priority_task_woken || higher_priority_task_woken_trace);
#else
	portYIELD_FROM_ISR(higher_priority_task_woken);
#endif
}

void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{
	usb_serial_send_to_usb((uint8_t *)data, len, true, true);
}

#if ENABLE_DEBUG == 1
size_t debug_serial_debug_write(const char *buf, const size_t len)
{
	return usb_serial_send_to_usb((uint8_t *)buf, len, true, true) ? len : 0;
}

__attribute__((used)) int _write(const int file, const void *const ptr, const size_t len)
{
	(void)file;
#ifdef PLATFORM_HAS_DEBUG
	size_t bytes_written = 0;

	if (debug_bmp)
		bytes_written = debug_serial_debug_write(ptr, len);
	else
		bytes_written = SEGGER_RTT_Write(0, ptr, len);

	return (bytes_written == len) ? (int)bytes_written : -1;
#else
	(void)ptr;
	return len;
#endif
}
#endif