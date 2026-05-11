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
#include "uart_ex.h"

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "uart_bridge.h"
#include "usb_cdc.h"
#include "swo.h"

#include "gdb_packet.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

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
#define TRACESWO_TASK_NOTIFY_WAIT_PERIOD   (portMAX_DELAY)

#define TRACESWO_VENDOR_INTERFACE (0)

#define TRACESWO_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define TRACESWO_TASK_STACK_SIZE    (512)

/** \brief Number of entries in \ref s_trace_bindings (SWO uses one UART binding today). */
#define TRACE_BINDING_COUNT (1u)

/** \brief Index of the SWO UART slot inside \ref s_trace_bindings. */
#define TRACE_BINDING_IDX_SWO (0u)

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

static uint8_t rx_buf[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS][TRACESWO_RX_DMA_BUFFER_SIZE] = {0};

/* Control-block list for the chained RX DMA. Alignment matches the ring-wrap window
 * (count * sizeof(uint32_t)) used by channel_config_set_ring inside the bridge. */
static uint8_t *rx_dma_ctrl_block_info[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS + 1]
	__attribute__((aligned(TRACESWO_RX_DMA_NUMBER_OF_BUFFERS * sizeof(uint32_t))));

static uart_bridge_ctx_t s_trace_ctx;
static bool s_trace_bridge_initialized = false;

static bool traceswo_decoding = false;
static TaskHandle_t traceswo_task = NULL;

/** \brief Active SWO mode after the last \c swo_init / \c swo_deinit. */
swo_coding_e swo_current_mode = swo_none;

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

static void traceswo_update_led(void);
static bool traceswo_send_to_usb(uint8_t *data, size_t len, bool flush, bool allow_drop_buffer);

static uart_bridge_sink_result_e trace_sink(
	uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop);
static void trace_on_rx_active(uart_bridge_ctx_t *ctx);

static void traceswo_rx_uart_handler(void);
static void traceswo_thread(void *params);

/**********************************************************************************************************************
 * Private Data (config)
 **********************************************************************************************************************/

/**
 * Single hardware binding for the SWO UART.  Pin numbers are filled in
 * \ref traceswo_task_init from \c platform_get_target_pins(); the bridge treats this
 * array as read-only after that.  \c tdi is intentionally forced back to \c SIO when
 * the binding becomes active so the pin does not drive while \c uart0 is in SWO mode.
 *
 * The binding uses \c shared_irq = true because \c uart0 is shared with target-serial
 * on TDI/TDO; both modules register their own thunk via \c irq_add_shared_handler.
 */
static uart_bridge_binding_t s_trace_bindings[TRACE_BINDING_COUNT] = {
	[TRACE_BINDING_IDX_SWO] = {
		.uart = TRACESWO_UART,
		.uart_irq = TRACESWO_UART_IRQ,
		.shared_irq = true,
		.uart_isr = traceswo_rx_uart_handler,
		.pins = {
			[UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
			[UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_SIO},
		},
	},
};

static const uart_bridge_config_t s_trace_cfg = {
	.rx_buffers_base = (uint8_t *)rx_buf,
	.rx_buffer_size = TRACESWO_RX_DMA_BUFFER_SIZE,
	.rx_buffer_count = TRACESWO_RX_DMA_NUMBER_OF_BUFFERS,
	.rx_ctrl_block_info = rx_dma_ctrl_block_info,
	.rx_drop_threshold = TRACESWO_RX_DMA_DROP_BUFFER_THRESHOLD,
	.rx_int_fifo_level = TRACESWO_UART_RX_INT_FIFO_LEVEL,
	.rx_dma_baudrate_threshold = TRACESWO_RX_DMA_BAUDRATE_THRESHOLD,
	.rx_dma_min_timeout_ms = TRACESWO_RX_DMA_MIN_TIMEOUT,
	.rx_dma_max_timeout_ms = TRACESWO_RX_DMA_MAX_TIMEOUT,
	.tx_buffer = NULL,
	.tx_buffer_size = 0,
	.tx_dma_check_finished_period_ms = 0,
	.notif_rx_available = USB_CDC_NOTIF_SERIAL_RX_AVAILABLE,
	.notif_rx_timeout = USB_CDC_NOTIF_SERIAL_RX_TIMEOUT,
	.notif_tx_complete = USB_CDC_NOTIF_SERIAL_TX_COMPLETE,
	.rx_sink = trace_sink,
	.tx_source = NULL,
	.on_rx_active = trace_on_rx_active,
	/* SWO is intentionally non-evictable: a cooperative claim from another channel
	 * must fail until \c swo_deinit releases the UART explicitly. */
	.on_release_request = NULL,
	.bindings = s_trace_bindings,
	.bindings_count = TRACE_BINDING_COUNT,
	.timer_name = "TRACE_RX_TIMEOUT",
	.user_ctx = NULL,
};

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

UART_BRIDGE_DECLARE_ISR(traceswo_rx_uart_handler, s_trace_ctx)

static void traceswo_update_led(void)
{
	platform_set_serial_state(s_trace_ctx.rx_ongoing);
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

static uart_bridge_sink_result_e trace_sink(
	uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop)
{
	(void)ctx;

	if (traceswo_decoding) {
		if (traceswo_decode(data, (uint16_t)len, flush, allow_drop)) {
			return UART_BRIDGE_SINK_OK;
		}
		return UART_BRIDGE_SINK_RETRY;
	}

	if (traceswo_send_to_usb(data, len, flush, allow_drop)) {
		return UART_BRIDGE_SINK_OK;
	}
	return UART_BRIDGE_SINK_STALL;
}

static void trace_on_rx_active(uart_bridge_ctx_t *ctx)
{
	(void)ctx;
	traceswo_update_led();
}

static void traceswo_thread(void *params)
{
	(void)params;

	if (traceswo_task == NULL) {
		traceswo_task = xTaskGetCurrentTaskHandle();
	}

	if (s_trace_bridge_initialized == false) {
		uart_bridge_init(&s_trace_ctx, &s_trace_cfg, traceswo_task);
		s_trace_bridge_initialized = true;
	}

	uint32_t notification_value = 0;
	const uint32_t wait_time = TRACESWO_TASK_NOTIFY_WAIT_PERIOD;

	while (1) {
		if (xTaskNotifyWait(0, UINT32_MAX, &notification_value, wait_time) == pdPASS) {
			if (notification_value & USB_CDC_NOTIF_SERIAL_RX_AVAILABLE) {
				if (s_trace_ctx.rx_use_dma == false) {
					uart_bridge_rx_int_process(&s_trace_ctx);
				} else {
					uart_bridge_rx_dma_process_buffers(&s_trace_ctx);
				}
			}

			if ((notification_value & USB_CDC_NOTIF_SERIAL_RX_TIMEOUT) && (s_trace_ctx.rx_ongoing != false)) {
				if (s_trace_ctx.rx_use_dma == false) {
					uart_bridge_rx_int_finish(&s_trace_ctx);
				} else {
					uart_bridge_rx_dma_finish_receiving(&s_trace_ctx);
				}
			}
		}

		traceswo_update_led();
	}
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

void swo_init(swo_coding_e swo_mode, uint32_t baudrate, uint32_t itm_stream_bitmask)
{
	/* swo_deinit invokes uart_bridge_deinit_uart / uart_bridge_release, both of
	 * which take the bridge mutex; that cannot happen inside portENTER_CRITICAL,
	 * so the deinit runs at task priority. */
	swo_deinit(false);

	if (baudrate == 0) {
		baudrate = SWO_DEFAULT_BAUD;
	}

	assert(swo_mode == swo_nrz_uart);

	/* Claim the UART.  FORCE cooperatively evicts \c usb_serial when it currently
	 * holds \c TRACESWO_UART (SWO has priority).  The bridge applies the matching
	 * binding's GPIO functions and installs the shared UART-IRQ handler atomically
	 * with the ownership update, so no separate \c gpio_set_function / \c irq_*
	 * sequence is required here. */
	if (uart_bridge_try_claim(&s_trace_ctx, TRACESWO_UART, UART_BRIDGE_CLAIM_FORCE) == false) {
		return;
	}

	uart_bridge_configure_uart(&s_trace_ctx, baudrate, 8, 1, UART_PARITY_NONE);

	memset(rx_buf, 0x00, sizeof(rx_buf));

	traceswo_setmask(itm_stream_bitmask);
	traceswo_decoding = itm_stream_bitmask != 0;

	swo_current_mode = swo_mode;

	gdb_outf("Baudrate: %" PRIu32 " ", swo_get_baudrate());
}

void swo_deinit(bool deallocate)
{
	(void)deallocate;

	if (swo_current_mode != swo_none) {
		uart_bridge_deinit_uart(&s_trace_ctx);
		uart_bridge_release(&s_trace_ctx);
	}

	swo_current_mode = swo_none;
}

void traceswo_task_init(void)
{
	/* Materialise the board-specific pin numbers into the binding before the bridge
	 * task can call \ref uart_bridge_try_claim.  \ref UART_BRIDGE_BINDING_PIN_TX
	 * (tdo) is the UART RX line; \ref UART_BRIDGE_BINDING_PIN_RX (tdi) returns to SIO
	 * so the line stops driving when SWO takes over \c uart0 from a prior
	 * target-serial TDI/TDO session. */
	const platform_target_pins_t *const target_pins = platform_get_target_pins();

	s_trace_bindings[TRACE_BINDING_IDX_SWO].pins[UART_BRIDGE_BINDING_PIN_TX].gpio = (int)target_pins->tdo;
	s_trace_bindings[TRACE_BINDING_IDX_SWO].pins[UART_BRIDGE_BINDING_PIN_RX].gpio = (int)target_pins->tdi;

#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(traceswo_thread, "target_trace", TRACESWO_TASK_STACK_SIZE, NULL,
		PLATFORM_PRIORITY_NORMAL, TRACESWO_TASK_CORE_AFFINITY, &traceswo_task);
#else
	const BaseType_t result = xTaskCreate(
		traceswo_thread, "target_trace", TRACESWO_TASK_STACK_SIZE, NULL, PLATFORM_PRIORITY_NORMAL, &traceswo_task);
#endif
	assert(result == pdPASS);
}

uint32_t swo_get_baudrate(void)
{
	if (swo_current_mode == swo_nrz_uart) {
		return uart_ex_get_baudrate(TRACESWO_UART);
	}
	return 0;
}
