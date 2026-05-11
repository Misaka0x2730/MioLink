/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2026 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "uart_ex.h"
#include "dma_ex.h"

#include "FreeRTOS.h"
#include "atomic.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "uart_bridge.h"

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief One slot in the UART ownership table.
 */
typedef struct {
	uart_inst_t *uart;        /**< UART instance currently held in this slot, \c NULL when free. */
	uart_bridge_ctx_t *owner; /**< Context owning the slot, \c NULL when free. */
} uart_bridge_owner_slot_t;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief UART ownership table.
 *
 * Concurrency model:
 *  - Structural operations (\ref uart_bridge_try_claim, \ref uart_bridge_release,
 *    \ref uart_bridge_get_owner, \ref uart_bridge_configure_uart,
 *    \ref uart_bridge_deinit_uart, \ref uart_bridge_deinit) serialise table mutation
 *    via \ref s_bridge_mutex (\c xSemaphoreCreateMutexStatic).
 *  - The table is never read or written from an ISR, so no additional ISR-vs-task
 *    interlock is required.
 */
static uart_bridge_owner_slot_t s_owners[UART_BRIDGE_MAX_OWNERS] = {0};

/**
 * \brief List of contexts that participate in the \c DMA_IRQ_0 dispatcher.
 *
 * Filled lazily by \ref uart_bridge_init; each context occupies a single slot for life.
 *
 * The dispatcher (\ref uart_bridge_dma_irq0_handler) reads this array from ISR
 * context, so mutations from task context use a short \c portENTER_CRITICAL section
 * to prevent the dispatcher from observing a half-updated slot.
 */
static uart_bridge_ctx_t *s_registered[UART_BRIDGE_MAX_CONTEXTS] = {0};

/**
 * \brief Whether \ref uart_bridge_dma_irq0_handler has been installed on \c DMA_IRQ_0.
 *
 * Updated only under \c portENTER_CRITICAL together with \ref s_registered.
 */
static bool s_dma_irq_installed = false;

/**
 * \brief Bridge structural-operations mutex.
 *
 * Created lazily by \ref uart_bridge_init under a brief \c portENTER_CRITICAL guard,
 * so the very first call from any task safely publishes the handle to all subsequent
 * callers on both cores. \c NULL until that first call returns.
 */
static SemaphoreHandle_t s_bridge_mutex = NULL;

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

static void uart_bridge_ensure_mutex(void);
static void uart_bridge_lock(void);
static void uart_bridge_unlock(void);
static void uart_bridge_register_ctx(uart_bridge_ctx_t *ctx);
static void uart_bridge_unregister_ctx(uart_bridge_ctx_t *ctx);
static int uart_bridge_find_slot_for_uart(uart_inst_t *uart);
static int uart_bridge_find_free_slot(void);
static void uart_bridge_drop_owner_entries(uart_bridge_ctx_t *ctx);
static const uart_bridge_binding_t *uart_bridge_find_binding(
	const uart_bridge_config_t *cfg, uart_inst_t *uart);
static void uart_bridge_apply_binding_locked(const uart_bridge_binding_t *binding);
static void uart_bridge_revert_binding_locked(const uart_bridge_binding_t *binding);
static void uart_bridge_release_locked(uart_bridge_ctx_t *ctx);
static void uart_bridge_deinit_uart_locked(uart_bridge_ctx_t *ctx);
static void uart_bridge_rx_timeout_callback(TimerHandle_t timer);
static BaseType_t uart_bridge_rx_dma_start_receiving(uart_bridge_ctx_t *ctx);
static void uart_bridge_dma_irq0_handler(void);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

/**
 * \brief Lazily create \ref s_bridge_mutex under a short cross-core spinlock so the
 *        first task that calls \ref uart_bridge_init publishes the handle exactly once.
 *
 * \c xSemaphoreCreateMutex does not block and its heap allocation path is safe to
 * call inside \c portENTER_CRITICAL on RP2040 (heap_4 uses a spinlock-protected
 * block list that is itself guarded by \c portENTER_CRITICAL internally).
 */
static void uart_bridge_ensure_mutex(void)
{
	portENTER_CRITICAL();
	if (s_bridge_mutex == NULL) {
		s_bridge_mutex = xSemaphoreCreateMutex();
	}
	portEXIT_CRITICAL();
}

/**
 * \brief Acquire the bridge structural-operations mutex.
 *
 * No-op when the mutex has not been created yet (i.e. before any
 * \ref uart_bridge_init has finished), which lets defensive callers from
 * pre-scheduler code paths exit gracefully.
 */
static void uart_bridge_lock(void)
{
	if (s_bridge_mutex != NULL) {
		xSemaphoreTake(s_bridge_mutex, portMAX_DELAY);
	}
}

/**
 * \brief Release the bridge structural-operations mutex; mirror of \ref uart_bridge_lock.
 */
static void uart_bridge_unlock(void)
{
	if (s_bridge_mutex != NULL) {
		xSemaphoreGive(s_bridge_mutex);
	}
}

static void uart_bridge_register_ctx(uart_bridge_ctx_t *ctx)
{
	for (uint32_t i = 0; i < UART_BRIDGE_MAX_CONTEXTS; i++) {
		if (s_registered[i] == ctx) {
			return;
		}
	}

	for (uint32_t i = 0; i < UART_BRIDGE_MAX_CONTEXTS; i++) {
		if (s_registered[i] == NULL) {
			s_registered[i] = ctx;
			return;
		}
	}

	/* Configuration error: bumping UART_BRIDGE_MAX_CONTEXTS is required. */
	assert(false);
}

static void uart_bridge_unregister_ctx(uart_bridge_ctx_t *ctx)
{
	for (uint32_t i = 0; i < UART_BRIDGE_MAX_CONTEXTS; i++) {
		if (s_registered[i] == ctx) {
			s_registered[i] = NULL;
			return;
		}
	}
}

static int uart_bridge_find_slot_for_uart(uart_inst_t *uart)
{
	for (int i = 0; i < (int)UART_BRIDGE_MAX_OWNERS; i++) {
		if (s_owners[i].uart == uart) {
			return i;
		}
	}
	return -1;
}

static int uart_bridge_find_free_slot(void)
{
	for (int i = 0; i < (int)UART_BRIDGE_MAX_OWNERS; i++) {
		if ((s_owners[i].uart == NULL) && (s_owners[i].owner == NULL)) {
			return i;
		}
	}
	return -1;
}

static void uart_bridge_drop_owner_entries(uart_bridge_ctx_t *ctx)
{
	for (uint32_t i = 0; i < UART_BRIDGE_MAX_OWNERS; i++) {
		if (s_owners[i].owner == ctx) {
			s_owners[i].uart = NULL;
			s_owners[i].owner = NULL;
		}
	}
}

/**
 * \brief Look up the binding describing the GPIO + IRQ wiring for \a uart inside \a cfg.
 *
 * \return Pointer to the matching binding, or \c NULL when \a uart is not declared
 *         by the owner.
 */
static const uart_bridge_binding_t *uart_bridge_find_binding(
	const uart_bridge_config_t *cfg, uart_inst_t *uart)
{
	if ((cfg == NULL) || (cfg->bindings == NULL) || (uart == NULL)) {
		return NULL;
	}
	for (uint32_t i = 0; i < cfg->bindings_count; i++) {
		if (cfg->bindings[i].uart == uart) {
			return &cfg->bindings[i];
		}
	}
	return NULL;
}

/**
 * \brief Apply a binding's GPIO functions and install its UART-IRQ handler.
 *
 * Called from \ref uart_bridge_try_claim under the bridge mutex when the context
 * transitions to a new UART.  The IRQ-line install races with hardware IRQ delivery
 * on the other core, so the actual register writes happen under a brief
 * \c portENTER_CRITICAL section.
 */
static void uart_bridge_apply_binding_locked(const uart_bridge_binding_t *binding)
{
	if (binding == NULL) {
		return;
	}

	portENTER_CRITICAL();

	for (uint32_t i = 0; i < UART_BRIDGE_MAX_PINS_PER_BINDING; i++) {
		if (binding->pins[i].gpio >= 0) {
			gpio_set_function((uint)binding->pins[i].gpio, binding->pins[i].function);
		}
	}

	if (binding->uart_isr != NULL) {
		if (binding->shared_irq == false) {
			/* Drop any stale exclusive handler so irq_set_exclusive_handler does not
			 * fault on a leftover registration from a previous module that may have
			 * forgotten to remove its handler. */
			const irq_handler_t cur = irq_get_exclusive_handler(binding->uart_irq);
			if (cur != NULL) {
				irq_remove_handler(binding->uart_irq, cur);
			}
			irq_set_exclusive_handler(binding->uart_irq, binding->uart_isr);
		} else {
			irq_add_shared_handler(binding->uart_irq, binding->uart_isr,
				PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
		}
		irq_set_enabled(binding->uart_irq, true);
	}

	portEXIT_CRITICAL();
}

/**
 * \brief Revert a previously applied binding: remove its UART-IRQ handler and return
 *        its GPIO pins to \c GPIO_FUNC_SIO.
 *
 * Called from \ref uart_bridge_deinit_uart_locked (so the deinit and reconfigure
 * paths share the teardown), under the bridge mutex.  Exclusive bindings additionally
 * mask the IRQ line in the NVIC; shared bindings leave the line alone because other
 * peers (e.g. SWO and target-serial on TDI/TDO) may still own it.
 */
static void uart_bridge_revert_binding_locked(const uart_bridge_binding_t *binding)
{
	if (binding == NULL) {
		return;
	}

	portENTER_CRITICAL();

	if (binding->uart_isr != NULL) {
		if (binding->shared_irq == false) {
			irq_set_enabled(binding->uart_irq, false);
			const irq_handler_t cur = irq_get_exclusive_handler(binding->uart_irq);
			if (cur == binding->uart_isr) {
				irq_remove_handler(binding->uart_irq, binding->uart_isr);
			}
		} else {
			/* Shared chain: just remove our entry.  The SDK keeps the IRQ line
			 * enabled while other handlers remain attached. */
			irq_remove_handler(binding->uart_irq, binding->uart_isr);
		}
	}

	/* Return the binding's GPIOs to SIO so the next owner starts from a clean state
	 * (and so an idle TDI/TDO pair does not keep driving lines as a peripheral). */
	for (uint32_t i = 0; i < UART_BRIDGE_MAX_PINS_PER_BINDING; i++) {
		if (binding->pins[i].gpio >= 0) {
			gpio_set_function((uint)binding->pins[i].gpio, GPIO_FUNC_SIO);
		}
	}

	portEXIT_CRITICAL();
}

static void uart_bridge_rx_timeout_callback(TimerHandle_t timer)
{
	uart_bridge_ctx_t *ctx = (uart_bridge_ctx_t *)pvTimerGetTimerID(timer);
	if (ctx == NULL) {
		return;
	}

	/* INT mode uses the UART RX-timeout interrupt; the FreeRTOS timer fires only
	 * for the DMA path. The additional rx_ongoing guard handles the case where the
	 * timer-service task delivers a callback that was already in flight while a
	 * concurrent uart_bridge_deinit_uart / configure_uart had reset the channel
	 * state (xTimerStop is asynchronous w.r.t. the timer-service task). */
	if (ctx->rx_use_dma && ctx->rx_ongoing) {
		xTaskNotify(ctx->owner_task, ctx->cfg->notif_rx_timeout, eSetBits);
	}
}

static BaseType_t uart_bridge_rx_dma_start_receiving(uart_bridge_ctx_t *ctx)
{
	assert(ctx->rx_ongoing == false);

	ctx->rx_ongoing = true;

	uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, false, false);
	uart_ex_clear_rx_and_rx_timeout_irq_flags(ctx->uart);
	uart_ex_set_dma_req_enabled(ctx->uart, true, ctx->cfg->tx_buffer != NULL);

	dma_ex_set_channel_enabled((uint32_t)ctx->rx_dma_channel, true, false);

	dma_channel_acknowledge_irq0((uint)ctx->rx_dma_channel);
	dma_ex_channel_set_irq_enabled((uint32_t)ctx->rx_dma_channel, 0);

	BaseType_t higher_priority_task_woken = pdFALSE;
	xTimerResetFromISR(ctx->rx_timeout_timer, &higher_priority_task_woken);
	return higher_priority_task_woken;
}

static void uart_bridge_dma_irq0_handler(void)
{
	traceISR_ENTER();

	BaseType_t higher_priority_task_woken = pdFALSE;

	for (uint32_t i = 0; i < UART_BRIDGE_MAX_CONTEXTS; i++) {
		uart_bridge_ctx_t *ctx = s_registered[i];
		if (ctx == NULL) {
			continue;
		}

		if ((ctx->tx_dma_channel >= 0) && dma_channel_get_irq0_status((uint)ctx->tx_dma_channel)) {
			dma_ex_channel_set_irq_disabled((uint32_t)ctx->tx_dma_channel, 0);
			dma_channel_acknowledge_irq0((uint)ctx->tx_dma_channel);
			xTaskNotifyFromISR(
				ctx->owner_task, ctx->cfg->notif_tx_complete, eSetBits, &higher_priority_task_woken);
		}

		if ((ctx->rx_dma_channel >= 0) && dma_channel_get_irq0_status((uint)ctx->rx_dma_channel)) {
			dma_channel_acknowledge_irq0((uint)ctx->rx_dma_channel);

			ctx->rx_dma_buffer_full_mask |= (1UL << ctx->rx_dma_current_buffer);
			if (++ctx->rx_dma_current_buffer >= ctx->cfg->rx_buffer_count) {
				ctx->rx_dma_current_buffer = 0;
			}

			xTaskNotifyFromISR(
				ctx->owner_task, ctx->cfg->notif_rx_available, eSetBits, &higher_priority_task_woken);
		}
	}

	portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

void uart_bridge_init(uart_bridge_ctx_t *ctx, const uart_bridge_config_t *cfg, TaskHandle_t owner_task)
{
	assert(ctx != NULL);
	assert(cfg != NULL);
	assert(cfg->rx_buffers_base != NULL);
	assert(cfg->rx_buffer_size > 0);
	assert(cfg->rx_buffer_count > 0);
	assert(cfg->rx_buffer_count <= 32);
	assert(cfg->rx_ctrl_block_info != NULL);
	assert(cfg->rx_sink != NULL);
	assert(cfg->bindings != NULL);
	assert(cfg->bindings_count > 0);

	ctx->cfg = cfg;
	ctx->owner_task = owner_task;

	/* Hardware stays detached until the first uart_bridge_try_claim: the bridge
	 * does not pre-configure any binding here, and the ownership table is left
	 * untouched so a subsequent claim takes the full setup path. */
	ctx->uart = NULL;
	ctx->uart_irq = 0;

	ctx->rx_dma_channel = dma_claim_unused_channel(true);
	ctx->rx_dma_ctrl_channel = dma_claim_unused_channel(true);
	ctx->tx_dma_channel = (cfg->tx_buffer != NULL) ? dma_claim_unused_channel(true) : -1;

	ctx->rx_use_dma = false;
	ctx->rx_ongoing = false;
	ctx->rx_int_buf_pos = 0;
	ctx->rx_dma_buffer_full_mask = 0;
	ctx->rx_dma_current_buffer = 0;
	ctx->rx_dma_next_buffer_to_send = 0;
	ctx->tx_ongoing = false;
	ctx->tx_dma_finished = false;

	for (uint32_t i = 0; i < cfg->rx_buffer_count; i++) {
		cfg->rx_ctrl_block_info[i] = cfg->rx_buffers_base + i * cfg->rx_buffer_size;
	}
	cfg->rx_ctrl_block_info[cfg->rx_buffer_count] = NULL;

	const TickType_t timer_period_ticks = pdMS_TO_TICKS(cfg->rx_dma_max_timeout_ms);
	ctx->rx_timeout_timer =
		xTimerCreate(cfg->timer_name, timer_period_ticks, pdFALSE, ctx, uart_bridge_rx_timeout_callback);
	assert(ctx->rx_timeout_timer != NULL);

	uart_bridge_ensure_mutex();

	/* s_registered[] and s_dma_irq_installed are read by uart_bridge_dma_irq0_handler
	 * from ISR context, so the publish of a new entry plus the (one-shot) handler
	 * installation must be atomic with respect to that ISR. The bridge mutex would
	 * not suffice on its own; portENTER_CRITICAL is the right primitive here. */
	portENTER_CRITICAL();
	uart_bridge_register_ctx(ctx);

	if (s_dma_irq_installed == false) {
		irq_set_exclusive_handler(DMA_IRQ_0, uart_bridge_dma_irq0_handler);
		irq_set_enabled(DMA_IRQ_0, true);
		s_dma_irq_installed = true;
	}
	portEXIT_CRITICAL();
}

void uart_bridge_configure_uart(
	uart_bridge_ctx_t *ctx, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity)
{
	assert(ctx != NULL);
	assert(ctx->uart != NULL);

	/* Hold the bridge mutex across the entire reconfigure so a concurrent
	 * uart_bridge_try_claim cannot reassign ctx->uart mid-flight, and so the
	 * xTimerChangePeriod call below (which would be illegal inside a critical
	 * section) is properly serialised against other structural operations. */
	uart_bridge_lock();

	/* 1. Disable the hardware sources of UART and DMA interrupts first so the
	 *    ISR and DMA dispatcher cannot observe partial state during the
	 *    reconfigure. uart_ex_set_*_enabled writes the peripheral mask, which
	 *    blocks future IRQ assertions on both cores. */
	uart_ex_set_dma_req_enabled(ctx->uart, false, false);
	uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, false, false);
	uart_ex_set_int_fifo_levels(ctx->uart, 0, 0);

	dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_ctrl_channel, 0);
	dma_channel_abort((uint)ctx->rx_dma_ctrl_channel);
	dma_channel_acknowledge_irq0((uint)ctx->rx_dma_ctrl_channel);

	dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_channel, 0);
	dma_channel_abort((uint)ctx->rx_dma_channel);
	dma_channel_acknowledge_irq0((uint)ctx->rx_dma_channel);

	if (ctx->tx_dma_channel >= 0) {
		dma_ex_channel_set_irq_disabled((uint32_t)ctx->tx_dma_channel, 0);
		dma_channel_abort((uint)ctx->tx_dma_channel);
		dma_channel_acknowledge_irq0((uint)ctx->tx_dma_channel);
	}

	/* Stop the RX idle timer before changing channel state so a stale callback
	 * cannot fire after the reset below; see R4 in the plan. */
	xTimerStop(ctx->rx_timeout_timer, 0);

	/* 2. With hardware IRQ sources masked, briefly take the FreeRTOS spinlock to
	 *    flush any UART/DMA ISR that was already running on the other core
	 *    (those ISRs take the same spinlock via xTaskNotifyFromISR / xTimerResetFromISR).
	 *    After this point the ctx fields can be reset without a TOCTOU window. */
	portENTER_CRITICAL();
	ctx->rx_int_buf_pos = 0;
	ctx->rx_ongoing = false;
	ctx->rx_dma_buffer_full_mask = 0;
	ctx->rx_dma_current_buffer = 0;
	ctx->rx_dma_next_buffer_to_send = 0;
	ctx->tx_ongoing = false;
	ctx->tx_dma_finished = false;
	portEXIT_CRITICAL();

	uart_init(ctx->uart, baudrate);
	uart_set_format(ctx->uart, data_bits, stop_bits, parity);

	if (ctx->tx_dma_channel >= 0) {
		dma_channel_config tx_config = dma_channel_get_default_config((uint)ctx->tx_dma_channel);
		channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
		channel_config_set_read_increment(&tx_config, true);
		channel_config_set_write_increment(&tx_config, false);
		channel_config_set_dreq(&tx_config, uart_get_dreq(ctx->uart, true));

		dma_channel_configure((uint)ctx->tx_dma_channel, &tx_config, uart_ex_get_dr_address(ctx->uart),
			ctx->cfg->tx_buffer, ctx->cfg->tx_buffer_size, false);
	}

	if (baudrate >= ctx->cfg->rx_dma_baudrate_threshold) {
		/* RX DMA path. */
		dma_channel_config rx_ctrl_config = dma_channel_get_default_config((uint)ctx->rx_dma_ctrl_channel);
		channel_config_set_transfer_data_size(&rx_ctrl_config, DMA_SIZE_32);
		channel_config_set_read_increment(&rx_ctrl_config, true);
		channel_config_set_write_increment(&rx_ctrl_config, false);
		channel_config_set_high_priority(&rx_ctrl_config, true);
		/* Ring read pointer over the rx_ctrl_block_info array (count + sentinel). */
		channel_config_set_ring(&rx_ctrl_config, false, 7);

		dma_channel_configure((uint)ctx->rx_dma_ctrl_channel, &rx_ctrl_config,
			dma_ex_get_al2_write_addr_trig((uint32_t)ctx->rx_dma_channel),
			(const volatile void *)ctx->cfg->rx_ctrl_block_info, 1, false);

		dma_channel_config rx_config = dma_channel_get_default_config((uint)ctx->rx_dma_channel);
		channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
		channel_config_set_read_increment(&rx_config, false);
		channel_config_set_write_increment(&rx_config, true);
		channel_config_set_dreq(&rx_config, uart_get_dreq(ctx->uart, false));
		channel_config_set_high_priority(&rx_config, true);
		channel_config_set_chain_to(&rx_config, (uint)ctx->rx_dma_ctrl_channel);

		dma_channel_configure((uint)ctx->rx_dma_channel, &rx_config, ctx->cfg->rx_buffers_base,
			uart_ex_get_dr_address(ctx->uart), ctx->cfg->rx_buffer_size, false);

		ctx->rx_use_dma = true;

		dma_ex_set_channel_enabled((uint32_t)ctx->rx_dma_channel, false, false);
		dma_ex_channel_set_irq_enabled((uint32_t)ctx->rx_dma_channel, 0);

		dma_channel_set_read_addr(
			(uint)ctx->rx_dma_ctrl_channel, (void *)ctx->cfg->rx_ctrl_block_info, true);

		/* Time to fill 2 RX buffers; clamp into [min, max] ms. */
		uint32_t timer_period = (ctx->cfg->rx_buffer_size * 2U * 1000U);
		timer_period /= (baudrate / 10U); /* 10 bit-times per byte (start + 8 data + stop). */
		if (timer_period < ctx->cfg->rx_dma_min_timeout_ms) {
			timer_period = ctx->cfg->rx_dma_min_timeout_ms;
		} else if (timer_period > ctx->cfg->rx_dma_max_timeout_ms) {
			timer_period = ctx->cfg->rx_dma_max_timeout_ms;
		}

		xTimerChangePeriod(ctx->rx_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

		uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, true, true);
	} else {
		/* RX INT path. */
		ctx->rx_use_dma = false;

		/* Set RX FIFO trigger level to 1/2 (FIFO holds 32 bytes; 1/2 = level 2). */
		uart_ex_set_int_fifo_levels(ctx->uart, 2, 0);
		uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, true, true);
	}

	uart_bridge_unlock();
}

void uart_bridge_rx_int_process(uart_bridge_ctx_t *ctx)
{
	if (ctx->rx_int_buf_pos > 0) {
		const uart_bridge_sink_result_e result = ctx->cfg->rx_sink(
			ctx, ctx->cfg->rx_buffers_base, ctx->rx_int_buf_pos, UART_BRIDGE_SINK_NO_FLUSH,
			UART_BRIDGE_SINK_ALLOW_DROP);
		if (result == UART_BRIDGE_SINK_OK) {
			ctx->rx_int_buf_pos = 0;
		}
		ctx->rx_ongoing = true;
	}

	uart_ex_set_rx_irq_enabled(ctx->uart, true);
}

void uart_bridge_rx_int_finish(uart_bridge_ctx_t *ctx)
{
	const uint32_t total_size = ctx->cfg->rx_buffer_size * ctx->cfg->rx_buffer_count;

	if (ctx->rx_int_buf_pos > 0) {
		ctx->cfg->rx_sink(ctx, ctx->cfg->rx_buffers_base, ctx->rx_int_buf_pos,
			UART_BRIDGE_SINK_NO_FLUSH, UART_BRIDGE_SINK_ALLOW_DROP);
		ctx->rx_int_buf_pos = 0;
	}

	while (uart_ex_is_rx_fifo_empty(ctx->uart) == false) {
		ctx->cfg->rx_buffers_base[ctx->rx_int_buf_pos++] = uart_ex_read(ctx->uart);
		if (ctx->rx_int_buf_pos >= total_size) {
			ctx->rx_int_buf_pos = 0;
		}
	}

	if (ctx->rx_int_buf_pos > 0) {
		ctx->cfg->rx_sink(ctx, ctx->cfg->rx_buffers_base, ctx->rx_int_buf_pos,
			UART_BRIDGE_SINK_FLUSH, UART_BRIDGE_SINK_ALLOW_DROP);
		ctx->rx_int_buf_pos = 0;
	}

	ctx->rx_ongoing = false;
	uart_ex_set_rx_timeout_irq_enabled(ctx->uart, true);
}

void uart_bridge_rx_dma_process_buffers(uart_bridge_ctx_t *ctx)
{
	xTimerReset(ctx->rx_timeout_timer, 0);

	while (1) {
		const uint32_t buffer_state = ctx->rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1UL << ctx->rx_dma_next_buffer_to_send);
		const bool allow_drop = (__builtin_popcount(buffer_state) >= (int)ctx->cfg->rx_drop_threshold);

		if ((buffer_state & buffer_bit) == 0) {
			break;
		}

		uint8_t *const buf =
			ctx->cfg->rx_buffers_base + ((size_t)ctx->rx_dma_next_buffer_to_send * ctx->cfg->rx_buffer_size);
		const uart_bridge_sink_result_e result =
			ctx->cfg->rx_sink(ctx, buf, ctx->cfg->rx_buffer_size, UART_BRIDGE_SINK_NO_FLUSH, allow_drop);

		if (result == UART_BRIDGE_SINK_OK) {
			Atomic_AND_u32(&ctx->rx_dma_buffer_full_mask, ~buffer_bit);
			if (++ctx->rx_dma_next_buffer_to_send >= ctx->cfg->rx_buffer_count) {
				ctx->rx_dma_next_buffer_to_send = 0;
			}
		} else if (result == UART_BRIDGE_SINK_RETRY) {
			xTimerReset(ctx->rx_timeout_timer, 0);
			vTaskDelay(pdMS_TO_TICKS(1));
		} else {
			break; /* UART_BRIDGE_SINK_STALL */
		}
	}
}

void uart_bridge_rx_dma_finish_receiving(uart_bridge_ctx_t *ctx)
{
	assert(ctx->rx_ongoing != false);

	uart_ex_set_dma_req_enabled(ctx->uart, false, ctx->cfg->tx_buffer != NULL);

	dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_ctrl_channel, 0);
	dma_channel_abort((uint)ctx->rx_dma_ctrl_channel);
	dma_channel_acknowledge_irq0((uint)ctx->rx_dma_ctrl_channel);

	const uint32_t current_buffer = ctx->rx_dma_current_buffer;

	if (++ctx->rx_dma_current_buffer >= ctx->cfg->rx_buffer_count) {
		ctx->rx_dma_current_buffer = 0;
	}

	ctx->rx_ongoing = false;

	xTimerStop(ctx->rx_timeout_timer, 0);

	const uint32_t remaining = dma_ex_get_trans_count((uint32_t)ctx->rx_dma_channel);
	const uint32_t data_in_buffer = ctx->cfg->rx_buffer_size - remaining;

	dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_channel, 0);
	dma_ex_set_chain_to((uint32_t)ctx->rx_dma_channel, (uint32_t)ctx->rx_dma_channel);
	dma_channel_abort((uint)ctx->rx_dma_channel);
	dma_channel_acknowledge_irq0((uint)ctx->rx_dma_channel);
	dma_ex_set_chain_to((uint32_t)ctx->rx_dma_channel, (uint32_t)ctx->rx_dma_ctrl_channel);

	dma_channel_set_read_addr((uint)ctx->rx_dma_ctrl_channel,
		(void *)(ctx->cfg->rx_ctrl_block_info + ctx->rx_dma_current_buffer), true);
	uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, true, true);

	while (1) {
		const uint32_t buffer_state = ctx->rx_dma_buffer_full_mask;
		const uint32_t buffer_bit = (1UL << ctx->rx_dma_next_buffer_to_send);
		if ((buffer_state & buffer_bit) == 0) {
			break;
		}

		uint8_t *const buf =
			ctx->cfg->rx_buffers_base + ((size_t)ctx->rx_dma_next_buffer_to_send * ctx->cfg->rx_buffer_size);
		ctx->cfg->rx_sink(ctx, buf, ctx->cfg->rx_buffer_size,
			UART_BRIDGE_SINK_NO_FLUSH, UART_BRIDGE_SINK_ALLOW_DROP);

		Atomic_AND_u32(&ctx->rx_dma_buffer_full_mask, ~buffer_bit);
		if (++ctx->rx_dma_next_buffer_to_send >= ctx->cfg->rx_buffer_count) {
			ctx->rx_dma_next_buffer_to_send = 0;
		}
	}

	if ((current_buffer + 1U) >= ctx->cfg->rx_buffer_count) {
		ctx->rx_dma_next_buffer_to_send = 0;
	} else {
		ctx->rx_dma_next_buffer_to_send = current_buffer + 1U;
	}

	uint8_t *const tail_buf =
		ctx->cfg->rx_buffers_base + ((size_t)current_buffer * ctx->cfg->rx_buffer_size);
	ctx->cfg->rx_sink(ctx, tail_buf, data_in_buffer, UART_BRIDGE_SINK_FLUSH, UART_BRIDGE_SINK_ALLOW_DROP);
}

void uart_bridge_tx_dma_send(uart_bridge_ctx_t *ctx)
{
	if (ctx->tx_dma_channel < 0) {
		return;
	}
	if (ctx->cfg->tx_source == NULL) {
		return;
	}

	const size_t read_count = ctx->cfg->tx_source(ctx, ctx->cfg->tx_buffer, ctx->cfg->tx_buffer_size);

	if (read_count != 0) {
		dma_channel_acknowledge_irq0((uint)ctx->tx_dma_channel);
		dma_ex_channel_set_irq_disabled((uint32_t)ctx->tx_dma_channel, 0);

		dma_channel_set_read_addr((uint)ctx->tx_dma_channel, ctx->cfg->tx_buffer, false);
		dma_channel_set_write_addr((uint)ctx->tx_dma_channel, uart_ex_get_dr_address(ctx->uart), false);
		dma_channel_set_trans_count((uint)ctx->tx_dma_channel, read_count, true);

		ctx->tx_ongoing = true;

		dma_ex_channel_set_irq_enabled((uint32_t)ctx->tx_dma_channel, 0);
	} else if (ctx->tx_ongoing) {
		ctx->tx_dma_finished = true;
	}
}

bool uart_bridge_tx_dma_check_finished(uart_bridge_ctx_t *ctx)
{
	if (ctx->tx_dma_channel < 0) {
		return true;
	}
	if (ctx->uart == NULL) {
		return true;
	}
	return !uart_ex_is_transmitting(ctx->uart);
}

void uart_bridge_uart_isr_handler(uart_bridge_ctx_t *ctx)
{
	traceISR_ENTER();

	const uint32_t uart_int_status = uart_ex_get_int_status(ctx->uart);
	assert(uart_int_status != 0);

	uint32_t notify_bits = 0;
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (ctx->rx_use_dma == false) {
		const uint32_t total_size = ctx->cfg->rx_buffer_size * ctx->cfg->rx_buffer_count;

		if (uart_int_status & RP_UART_INT_RX_BITS) {
			for (uint32_t i = 0; i < (ctx->cfg->rx_int_fifo_level - 1); i++) {
				if (uart_ex_is_rx_fifo_empty(ctx->uart)) {
					break;
				}

				ctx->cfg->rx_buffers_base[ctx->rx_int_buf_pos] = uart_ex_read(ctx->uart);
				if (++ctx->rx_int_buf_pos >= total_size) {
					ctx->rx_int_buf_pos = 0;
				}
			}

			uart_ex_clear_rx_irq_flag(ctx->uart);
			uart_ex_set_rx_irq_enabled(ctx->uart, false);
			notify_bits |= ctx->cfg->notif_rx_available;
		}

		if (uart_int_status & RP_UART_INT_RX_TIMEOUT_BITS) {
			uart_ex_clear_rx_timeout_irq_flag(ctx->uart);
			uart_ex_set_rx_timeout_irq_enabled(ctx->uart, false);
			notify_bits |= ctx->cfg->notif_rx_timeout;
		}

		xTaskNotifyFromISR(ctx->owner_task, notify_bits, eSetBits, &higher_priority_task_woken);
	} else {
		higher_priority_task_woken = uart_bridge_rx_dma_start_receiving(ctx);
		uart_ex_clear_rx_and_rx_timeout_irq_flags(ctx->uart);

		if (ctx->cfg->on_rx_active != NULL) {
			ctx->cfg->on_rx_active(ctx);
		}
	}

	portYIELD_FROM_ISR(higher_priority_task_woken);
}

bool uart_bridge_try_claim(uart_bridge_ctx_t *ctx, uart_inst_t *new_uart, bool force)
{
	if ((ctx == NULL) || (new_uart == NULL)) {
		return false;
	}

	/* Fast path: ctx already owns new_uart.  The structural state read here is
	 * monotonic between try_claim / release calls on the same ctx and a stale-read
	 * is harmless (worst case we fall through to the locked path). */
	if (ctx->uart == new_uart) {
		return true;
	}

	/* Resolve the binding for the requested UART up front: the bridge needs the
	 * binding both for the IRQ line and for the GPIO + ISR install sequence below.
	 * A missing binding is a configuration error on the owner side. */
	const uart_bridge_binding_t *new_binding = uart_bridge_find_binding(ctx->cfg, new_uart);
	if (new_binding == NULL) {
		assert(false);
		return false;
	}

	uart_bridge_lock();

	int slot = uart_bridge_find_slot_for_uart(new_uart);
	uart_bridge_ctx_t *current_owner = (slot >= 0) ? s_owners[slot].owner : NULL;

	if ((current_owner != NULL) && (current_owner != ctx)) {
		/* The UART is owned by a different context. */
		if (!force) {
			uart_bridge_unlock();
			return false;
		}

		if ((current_owner->cfg == NULL) || (current_owner->cfg->on_release_request == NULL)) {
			uart_bridge_unlock();
			return false;
		}

		/* Drop the bridge mutex around the cooperative-release callback: the
		 * callback runs in the requester's task context and will itself call
		 * into the bridge (typically uart_bridge_deinit_uart), which needs to
		 * take this same mutex.  Since the mutex is non-recursive we must
		 * release it here. */
		uart_bridge_unlock();

		const bool released = current_owner->cfg->on_release_request(current_owner);
		if (!released) {
			return false;
		}

		uart_bridge_lock();

		/* Re-check the slot: another task on the other core may have claimed
		 * the UART while we were outside the mutex. */
		slot = uart_bridge_find_slot_for_uart(new_uart);
		uart_bridge_ctx_t *post_owner = (slot >= 0) ? s_owners[slot].owner : NULL;

		if ((post_owner != NULL) && (post_owner != current_owner) && (post_owner != ctx)) {
			/* Another context beat us to it; leave it alone. */
			uart_bridge_unlock();
			return false;
		}

		/* Drop the released owner's entries (on_release_request does not touch
		 * the ownership table). */
		if (post_owner == current_owner) {
			uart_bridge_drop_owner_entries(current_owner);
			current_owner->uart = NULL;
			current_owner->uart_irq = 0;
		}
	}

	/* Tear down the hardware on our previously-bound UART, if any.  This removes
	 * the old binding's UART-IRQ handler, returns its GPIO pins to SIO, and
	 * resets per-context runtime state so the subsequent uart_bridge_configure_uart
	 * starts from a known baseline. */
	if (ctx->uart != NULL) {
		uart_bridge_deinit_uart_locked(ctx);
	}
	/* Always drop any stale ownership entries for ctx so a single context never
	 * occupies more than one slot, regardless of prior pathological state. */
	uart_bridge_drop_owner_entries(ctx);
	ctx->uart = NULL;
	ctx->uart_irq = 0;

	slot = uart_bridge_find_slot_for_uart(new_uart);
	if (slot < 0) {
		slot = uart_bridge_find_free_slot();
	}
	if (slot < 0) {
		uart_bridge_unlock();
		return false;
	}

	s_owners[slot].uart = new_uart;
	s_owners[slot].owner = ctx;
	ctx->uart = new_uart;
	ctx->uart_irq = new_binding->uart_irq;

	/* Install the new binding's GPIO functions and UART-IRQ handler atomically
	 * with the ownership update; the UART peripheral itself is configured by the
	 * subsequent uart_bridge_configure_uart call. */
	uart_bridge_apply_binding_locked(new_binding);

	uart_bridge_unlock();
	return true;
}

/**
 * \brief Mutex-held helper for \ref uart_bridge_release.
 *
 * Caller must hold \ref s_bridge_mutex.
 */
static void uart_bridge_release_locked(uart_bridge_ctx_t *ctx)
{
	uart_bridge_drop_owner_entries(ctx);
	ctx->uart = NULL;
	ctx->uart_irq = 0;
}

void uart_bridge_release(uart_bridge_ctx_t *ctx)
{
	if (ctx == NULL) {
		return;
	}

	uart_bridge_lock();
	uart_bridge_release_locked(ctx);
	uart_bridge_unlock();
}

uart_bridge_ctx_t *uart_bridge_get_owner(uart_inst_t *uart)
{
	if (uart == NULL) {
		return NULL;
	}

	uart_bridge_ctx_t *owner = NULL;

	uart_bridge_lock();
	const int slot = uart_bridge_find_slot_for_uart(uart);
	if (slot >= 0) {
		owner = s_owners[slot].owner;
	}
	uart_bridge_unlock();

	return owner;
}

/**
 * \brief Mutex-held helper for \ref uart_bridge_deinit_uart.
 *
 * Caller must hold \ref s_bridge_mutex. The function is reused by
 * \ref uart_bridge_deinit to avoid re-locking on the same path.
 */
static void uart_bridge_deinit_uart_locked(uart_bridge_ctx_t *ctx)
{
	/* Stop the RX-idle timer first; xTimerStop with 0 timeout is non-blocking. */
	xTimerStop(ctx->rx_timeout_timer, 0);

	/* Disable and abort all DMA channels before touching the UART peripheral so
	 * no in-flight transfer can write to a buffer about to be invalidated. */
	if (ctx->rx_dma_ctrl_channel >= 0) {
		dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_ctrl_channel, 0);
		dma_channel_abort((uint)ctx->rx_dma_ctrl_channel);
		dma_channel_acknowledge_irq0((uint)ctx->rx_dma_ctrl_channel);
	}
	if (ctx->rx_dma_channel >= 0) {
		dma_ex_channel_set_irq_disabled((uint32_t)ctx->rx_dma_channel, 0);
		dma_channel_abort((uint)ctx->rx_dma_channel);
		dma_channel_acknowledge_irq0((uint)ctx->rx_dma_channel);
	}
	if (ctx->tx_dma_channel >= 0) {
		dma_ex_channel_set_irq_disabled((uint32_t)ctx->tx_dma_channel, 0);
		dma_channel_abort((uint)ctx->tx_dma_channel);
		dma_channel_acknowledge_irq0((uint)ctx->tx_dma_channel);
	}

	if (ctx->uart != NULL) {
		/* Mask UART's own interrupt generation before removing the handler so the
		 * UART cannot assert a new IRQ between the mask and the handler removal. */
		uart_ex_set_dma_req_enabled(ctx->uart, false, false);
		uart_ex_set_rx_and_timeout_irq_enabled(ctx->uart, false, false);
		uart_ex_clear_rx_and_rx_timeout_irq_flags(ctx->uart);

		/* Brief portENTER_CRITICAL window to flush any UART/DMA ISR still running
		 * on the other core. The ISR uses xTaskNotifyFromISR / xTimerResetFromISR
		 * which take the FreeRTOS spinlock that portENTER_CRITICAL waits on, so
		 * by the time we exit no ISR for this ctx can be in flight. */
		portENTER_CRITICAL();
		portEXIT_CRITICAL();

		/* Roll back the active binding: remove its UART-IRQ handler and return
		 * its GPIO pins to SIO so the next owner starts from a clean state. */
		uart_bridge_revert_binding_locked(uart_bridge_find_binding(ctx->cfg, ctx->uart));

		uart_deinit(ctx->uart);
	}

	/* Reset all runtime state so a subsequent uart_bridge_configure_uart starts
	 * from a known baseline. rx_dma_buffer_full_mask is also read by the DMA
	 * dispatcher ISR but the IRQ source is now disabled, so a plain store is OK. */
	ctx->rx_use_dma = false;
	ctx->rx_ongoing = false;
	ctx->rx_int_buf_pos = 0;
	ctx->rx_dma_buffer_full_mask = 0;
	ctx->rx_dma_current_buffer = 0;
	ctx->rx_dma_next_buffer_to_send = 0;
	ctx->tx_ongoing = false;
	ctx->tx_dma_finished = false;
}

void uart_bridge_deinit_uart(uart_bridge_ctx_t *ctx)
{
	if (ctx == NULL) {
		return;
	}

	uart_bridge_lock();
	uart_bridge_deinit_uart_locked(ctx);
	uart_bridge_unlock();
}

void uart_bridge_deinit(uart_bridge_ctx_t *ctx)
{
	if (ctx == NULL) {
		return;
	}

	uart_bridge_lock();

	uart_bridge_deinit_uart_locked(ctx);
	uart_bridge_release_locked(ctx);

	if (ctx->rx_timeout_timer != NULL) {
		xTimerDelete(ctx->rx_timeout_timer, portMAX_DELAY);
		ctx->rx_timeout_timer = NULL;
	}

	if (ctx->rx_dma_channel >= 0) {
		dma_channel_unclaim((uint)ctx->rx_dma_channel);
		ctx->rx_dma_channel = -1;
	}
	if (ctx->rx_dma_ctrl_channel >= 0) {
		dma_channel_unclaim((uint)ctx->rx_dma_ctrl_channel);
		ctx->rx_dma_ctrl_channel = -1;
	}
	if (ctx->tx_dma_channel >= 0) {
		dma_channel_unclaim((uint)ctx->tx_dma_channel);
		ctx->tx_dma_channel = -1;
	}

	/* Remove from the DMA dispatcher and tear down the shared IRQ if this was the
	 * last registered context. The window must be atomic w.r.t. the dispatcher,
	 * which runs in IRQ context; the bridge mutex alone is insufficient there. */
	portENTER_CRITICAL();
	uart_bridge_unregister_ctx(ctx);

	bool any_registered = false;
	for (uint32_t i = 0; i < UART_BRIDGE_MAX_CONTEXTS; i++) {
		if (s_registered[i] != NULL) {
			any_registered = true;
			break;
		}
	}

	if (!any_registered && s_dma_irq_installed) {
		irq_set_enabled(DMA_IRQ_0, false);
		irq_remove_handler(DMA_IRQ_0, uart_bridge_dma_irq0_handler);
		s_dma_irq_installed = false;
	}
	portEXIT_CRITICAL();

	uart_bridge_unlock();
}
