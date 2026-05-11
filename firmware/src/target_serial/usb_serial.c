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

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/uart.h"

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "uart_bridge.h"
#include "usb_cdc.h"
#include "usb_serial.h"

#if ENABLE_DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef ENABLE_RTT
#include "rtt.h"
#endif

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

/* UART RX Interrupt mode settings */
#define USB_SERIAL_UART_RX_INT_FIFO_LEVEL (16)

/* UART RX DMA mode settings */
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

/* UART TX DMA settings */
#define USB_SERIAL_UART_DMA_TX_BUFFER_SIZE           (256)
#define USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD (pdMS_TO_TICKS(2))

/* USB serial task settings */
#define USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD portMAX_DELAY

#define USB_SERIAL_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define USB_SERIAL_TASK_STACK_SIZE    (512)

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

static uint8_t uart_rx_buf[USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS][USB_SERIAL_UART_DMA_RX_BUFFER_SIZE] = {0};
static uint8_t uart_tx_dma_buf[USB_SERIAL_UART_DMA_TX_BUFFER_SIZE] = {0};

/* Control-block list for the chained RX DMA. Alignment matches the ring-wrap window
 * (count * sizeof(uint32_t)) used by channel_config_set_ring inside the bridge. */
static uint8_t *uart_dma_rx_ctrl_block_info[USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS + 1]
	__attribute__((aligned(USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS * sizeof(uint32_t))));

static uart_bridge_ctx_t s_serial_ctx;

static bool use_uart_on_tdi_tdo = false;

TaskHandle_t usb_uart_task = NULL;

/**
 * Slot indices into \ref s_serial_bindings.  The MAIN UART uses an exclusive UART-IRQ,
 * while the TDI/TDO UART shares \c uart0's IRQ line with SWO and therefore registers
 * the same thunk as a shared handler.
 */
#define SERIAL_BINDING_MAIN    (0u)
#define SERIAL_BINDING_TDI_TDO (1u)
#define SERIAL_BINDING_COUNT   (2u)

#ifdef ENABLE_RTT
extern void rtt_serial_receive_callback(void);
#endif

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

static uart_bridge_sink_result_e serial_sink(
	uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop);
static size_t serial_tx_source(uart_bridge_ctx_t *ctx, uint8_t *dst, size_t cap);
static void serial_on_rx_active(uart_bridge_ctx_t *ctx);
static bool serial_on_release_request(uart_bridge_ctx_t *ctx);

static void uart_rx_isr(void);
static void serial_update_config(cdc_line_coding_t *line_coding);
static void target_serial_thread(void *params);

/**********************************************************************************************************************
 * Private Data (config)
 **********************************************************************************************************************/

/**
 * Hardware bindings consumed by the bridge.  Pin numbers are filled at runtime in
 * \ref usb_serial_init from \c platform_get_target_pins(); the bridge treats this
 * array as read-only afterwards.  Index 0 (MAIN) is the channel's default binding
 * and uses an exclusive UART-IRQ; index 1 (TDI/TDO) shares \c uart0 with SWO and
 * therefore declares \c shared_irq = true.
 */
static uart_bridge_binding_t s_serial_bindings[SERIAL_BINDING_COUNT] = {
	[SERIAL_BINDING_MAIN] = {
		.uart = USB_SERIAL_UART_MAIN,
		.uart_irq = USB_SERIAL_UART_MAIN_IRQ,
		.shared_irq = false,
		.uart_isr = uart_rx_isr,
		.pins = {
			[UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
			[UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_UART},
		},
	},
	[SERIAL_BINDING_TDI_TDO] = {
		.uart = USB_SERIAL_UART_TDI_TDO,
		.uart_irq = USB_SERIAL_UART_TDI_TDO_IRQ,
		.shared_irq = true,
		.uart_isr = uart_rx_isr,
		.pins = {
			[UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
			[UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_UART},
		},
	},
};

static const uart_bridge_config_t s_serial_cfg = {
	.rx_buffers_base = (uint8_t *)uart_rx_buf,
	.rx_buffer_size = USB_SERIAL_UART_DMA_RX_BUFFER_SIZE,
	.rx_buffer_count = USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS,
	.rx_ctrl_block_info = uart_dma_rx_ctrl_block_info,
	.rx_drop_threshold = USB_SERIAL_UART_DMA_RX_DROP_BUFFER_THRESHOLD,
	.rx_int_fifo_level = USB_SERIAL_UART_RX_INT_FIFO_LEVEL,
	.rx_dma_baudrate_threshold = USB_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD,
	.rx_dma_min_timeout_ms = USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT,
	.rx_dma_max_timeout_ms = USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT,
	.tx_buffer = uart_tx_dma_buf,
	.tx_buffer_size = sizeof(uart_tx_dma_buf),
	.tx_dma_check_finished_period_ms = 2,
	.notif_rx_available = USB_CDC_NOTIF_SERIAL_RX_AVAILABLE,
	.notif_rx_timeout = USB_CDC_NOTIF_SERIAL_RX_TIMEOUT,
	.notif_tx_complete = USB_CDC_NOTIF_SERIAL_TX_COMPLETE,
	.rx_sink = serial_sink,
	.tx_source = serial_tx_source,
	.on_rx_active = serial_on_rx_active,
	.on_release_request = serial_on_release_request,
	.bindings = s_serial_bindings,
	.bindings_count = SERIAL_BINDING_COUNT,
	.timer_name = "SERIAL_UART_RX",
	.user_ctx = NULL,
};

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

UART_BRIDGE_DECLARE_ISR(uart_rx_isr, s_serial_ctx)

static uart_bridge_sink_result_e serial_sink(
	uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop)
{
	(void)ctx;
	if (usb_serial_send_to_usb(data, len, flush, allow_drop)) {
		return UART_BRIDGE_SINK_OK;
	}
	return UART_BRIDGE_SINK_STALL;
}

static size_t serial_tx_source(uart_bridge_ctx_t *ctx, uint8_t *dst, size_t cap)
{
	(void)ctx;
	if (tud_cdc_n_available(USB_CDC_TARGET_SERIAL) == 0) {
		return 0;
	}
	return tud_cdc_n_read(USB_CDC_TARGET_SERIAL, dst, cap);
}

static void serial_on_rx_active(uart_bridge_ctx_t *ctx)
{
	(void)ctx;
	usb_serial_update_led();
}

static bool serial_on_release_request(uart_bridge_ctx_t *ctx)
{
	/* Bridge-driven cooperative eviction: drop our hold on the UART so the
	 * requesting context (typically SWO) can claim it.  uart_bridge_deinit_uart
	 * removes the active binding's UART-IRQ handler, returns its GPIO pins to
	 * SIO, and resets our DMA / timer state. */
	uart_bridge_deinit_uart(ctx);

	/* Kick the serial task so its polling path re-attempts the claim after the
	 * requesting owner has finished its setup. */
	if (usb_uart_task != NULL) {
		xTaskNotify(usb_uart_task, USB_CDC_NOTIF_DUMMY, eSetBits);
	}
	return true;
}

static void serial_update_config(cdc_line_coding_t *line_coding)
{
	uint8_t stop_bits = 2;
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

	uart_inst_t *const desired_uart = use_uart_on_tdi_tdo ? USB_SERIAL_UART_TDI_TDO : USB_SERIAL_UART_MAIN;

	/* NO_FORCE: never evict the SWO owner; if it currently holds the contested UART
	 * we will retry from the polling section of target_serial_thread.  The bridge
	 * handles GPIO + UART-IRQ install/remove during the transition based on the
	 * binding declared in s_serial_cfg.bindings[]. */
	if (!uart_bridge_try_claim(&s_serial_ctx, desired_uart, UART_BRIDGE_CLAIM_NO_FORCE)) {
		return;
	}

	uart_bridge_configure_uart(&s_serial_ctx, line_coding->bit_rate, data_bits, stop_bits, parity);
}

static void target_serial_thread(void *params)
{
	(void)params;

	if (usb_uart_task == NULL) {
		usb_uart_task = xTaskGetCurrentTaskHandle();
	}

	uart_bridge_init(&s_serial_ctx, &s_serial_cfg, usb_uart_task);

	uint32_t notification_value = 0;
	uint32_t wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

	while (1) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, wait_time) == pdPASS) {
			if (notification_value & USB_CDC_NOTIF_LINE_CODING_UPDATE) {
				cdc_line_coding_t line_coding = {0};
				tud_cdc_n_get_line_coding(USB_CDC_TARGET_SERIAL, &line_coding);
				serial_update_config(&line_coding);
			}

			if (notification_value & USB_CDC_NOTIF_SERIAL_RX_AVAILABLE) {
				if (s_serial_ctx.rx_use_dma == false) {
					uart_bridge_rx_int_process(&s_serial_ctx);
				} else {
					uart_bridge_rx_dma_process_buffers(&s_serial_ctx);
				}
			}

			if ((notification_value & USB_CDC_NOTIF_SERIAL_RX_TIMEOUT) && (s_serial_ctx.rx_ongoing != false)) {
				if (s_serial_ctx.rx_use_dma == false) {
					uart_bridge_rx_int_finish(&s_serial_ctx);
				} else {
					uart_bridge_rx_dma_finish_receiving(&s_serial_ctx);
				}
			}

			if (notification_value & USB_CDC_NOTIF_SERIAL_TX_COMPLETE) {
				uart_bridge_tx_dma_send(&s_serial_ctx);
			}

			if ((notification_value & USB_CDC_NOTIF_USB_RX_AVAILABLE) && (s_serial_ctx.tx_ongoing == false)) {
#ifdef ENABLE_RTT
				if (rtt_enabled) {
					rtt_serial_receive_callback();
				} else {
					uart_bridge_tx_dma_send(&s_serial_ctx);
				}
#else
				uart_bridge_tx_dma_send(&s_serial_ctx);
#endif
			}
		}

		if (s_serial_ctx.tx_dma_finished) {
			if (uart_bridge_tx_dma_check_finished(&s_serial_ctx)) {
				wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

				s_serial_ctx.tx_ongoing = false;
				s_serial_ctx.tx_dma_finished = false;
			} else {
				wait_time = USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD;
			}
		}

		uart_inst_t *const desired_uart =
			use_uart_on_tdi_tdo ? USB_SERIAL_UART_TDI_TDO : USB_SERIAL_UART_MAIN;
		if (s_serial_ctx.uart != desired_uart) {
			cdc_line_coding_t line_coding = {0};
			tud_cdc_n_get_line_coding(USB_CDC_TARGET_SERIAL, &line_coding);
			serial_update_config(&line_coding);
		}

		usb_serial_update_led();
	}
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

bool usb_serial_get_dtr(void)
{
	return (tud_cdc_n_get_line_state(USB_CDC_TARGET_SERIAL) & 0x01) != 0;
}

void usb_serial_update_led(void)
{
	if (tud_cdc_n_connected(USB_CDC_TARGET_SERIAL) == false) {
		platform_set_serial_state(false);
	} else {
		platform_set_serial_state(s_serial_ctx.rx_ongoing || s_serial_ctx.tx_ongoing);
	}
}

uint16_t usb_serial_get_available(void)
{
	return tud_cdc_n_write_available(USB_CDC_TARGET_SERIAL);
}

uint32_t usb_serial_read(uint8_t *data, const uint32_t buffer_size)
{
	return tud_cdc_n_read(USB_CDC_TARGET_SERIAL, data, buffer_size);
}

bool usb_serial_send_to_usb(uint8_t *data, const size_t len, bool flush, const bool allow_drop_buffer)
{
	bool result = false;
	const uint32_t write_available = tud_cdc_n_write_available(USB_CDC_TARGET_SERIAL);

	if (usb_serial_get_dtr() == false) {
		return true;
	}

	if (write_available < len) {
		if (allow_drop_buffer) {
			tud_cdc_n_write(USB_CDC_TARGET_SERIAL, data, write_available);
			result = true;
		}
	} else if (len > 0) {
		result = (tud_cdc_n_write(USB_CDC_TARGET_SERIAL, data, len) == len);
	} else {
		result = true;
		flush = true;
	}

	if (result && flush) {
		tud_cdc_n_write_flush(USB_CDC_TARGET_SERIAL);
	}

	return result;
}

void usb_serial_use_uart_on_tdi_tdo(const bool new_state)
{
	use_uart_on_tdi_tdo = new_state;
	xTaskNotify(usb_uart_task, USB_CDC_NOTIF_DUMMY, eSetBits);
}

bool usb_serial_uart_on_tdi_tdo_is_used(void)
{
	return use_uart_on_tdi_tdo;
}

void usb_serial_init(void)
{
	/* Materialise the board-specific pin numbers into the bindings before the bridge
	 * task can call \ref uart_bridge_try_claim.  The bindings array is treated as
	 * read-only by the bridge after this point. */
	const platform_target_pins_t *const target_pins = platform_get_target_pins();

	s_serial_bindings[SERIAL_BINDING_MAIN].pins[UART_BRIDGE_BINDING_PIN_TX].gpio = (int)target_pins->uart_tx;
	s_serial_bindings[SERIAL_BINDING_MAIN].pins[UART_BRIDGE_BINDING_PIN_RX].gpio = (int)target_pins->uart_rx;
	s_serial_bindings[SERIAL_BINDING_TDI_TDO].pins[UART_BRIDGE_BINDING_PIN_TX].gpio = (int)target_pins->tdo;
	s_serial_bindings[SERIAL_BINDING_TDI_TDO].pins[UART_BRIDGE_BINDING_PIN_RX].gpio = (int)target_pins->tdi;

#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(target_serial_thread, "target_uart", USB_SERIAL_TASK_STACK_SIZE,
		NULL, PLATFORM_PRIORITY_NORMAL, USB_SERIAL_TASK_CORE_AFFINITY, &usb_uart_task);
#else
	const BaseType_t result = xTaskCreate(target_serial_thread, "target_uart", USB_SERIAL_TASK_STACK_SIZE, NULL,
		PLATFORM_PRIORITY_NORMAL, &usb_uart_task);
#endif

	assert(result == pdPASS);
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
