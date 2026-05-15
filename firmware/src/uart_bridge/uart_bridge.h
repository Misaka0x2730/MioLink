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

#ifndef MIOLINK_UART_BRIDGE_H
#define MIOLINK_UART_BRIDGE_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

/**
 * \brief Maximum number of UART instances tracked by the ownership table.
 *
 * RP2040 has \c uart0 and \c uart1, so two slots are sufficient.
 */
#define UART_BRIDGE_MAX_OWNERS (2u)

/**
 * \brief Maximum number of \ref uart_bridge_ctx_t contexts that may register with the
 *        single \c DMA_IRQ_0 dispatcher.
 *
 * One context per owner module (target serial UART, async SWO).
 */
#define UART_BRIDGE_MAX_CONTEXTS (2u)

/**
 * \brief Maximum number of GPIO pins declared per \ref uart_bridge_binding_t.
 *
 * Two pins are enough for every currently supported owner: \c uart_tx + \c uart_rx
 * for the main UART, \c tdo + \c tdi for the TDI/TDO and SWO bindings.
 */
#define UART_BRIDGE_MAX_PINS_PER_BINDING (2u)

/**
 * \brief Slot index for the transmit-side UART GPIO in \ref uart_bridge_binding_t::pins.
 *
 * Main UART: \c uart_tx. Alternate bindings: JTAG mux line used as probe TX (typically \c tdo).
 */
#define UART_BRIDGE_BINDING_PIN_TX (0u)

/**
 * \brief Slot index for the receive-side UART GPIO in \ref uart_bridge_binding_t::pins.
 *
 * Main UART: \c uart_rx. Alternate bindings: mux line used as probe RX (\c tdi) or SIO for SWO (\c tdi).
 */
#define UART_BRIDGE_BINDING_PIN_RX (1u)

/**
 * \brief Pass to \ref uart_bridge_try_claim to allow cooperative eviction of the current
 *        UART owner via its \c on_release_request callback.
 */
#define UART_BRIDGE_CLAIM_FORCE (true)

/**
 * \brief Pass to \ref uart_bridge_try_claim to prohibit eviction: return \c false
 *        immediately when the UART is already owned by a different context.
 */
#define UART_BRIDGE_CLAIM_NO_FORCE (false)

/**
 * \brief Compile-time mapping from a UART instance to its NVIC IRQ line.
 *
 * Thin alias over Pico SDK's \c UART_IRQ_NUM that documents the bridge's
 * single source of truth for UART → IRQ resolution: owner modules no longer
 * declare per-binding IRQ numbers, and the bridge derives them from
 * \c binding->uart whenever it installs or enables its central UART
 * dispatcher.
 *
 * \param[in] uart UART instance (\c uart0 or \c uart1).
 */
#define UART_BRIDGE_UART_IRQ_NUM(uart) UART_IRQ_NUM(uart)

/**
 * \brief Default value for \ref uart_bridge_config_t::rx_int_fifo_level.
 *
 * Drains roughly half of the RP2040 UART RX FIFO (32 entries) per ISR in INT mode,
 * which balances ISR rate against the headroom left before a hardware overrun.
 */
#define UART_BRIDGE_DEFAULT_RX_INT_FIFO_LEVEL (16u)

/**
 * \brief Default value for \ref uart_bridge_config_t::rx_dma_baudrate_threshold.
 *
 * At and above this baudrate the bridge switches from the INT path to the DMA path
 * so the per-byte IRQ rate does not saturate the CPU.
 */
#define UART_BRIDGE_DEFAULT_RX_DMA_BAUDRATE_THRESHOLD (38400u)

/**
 * \brief Default lower clamp for \ref uart_bridge_config_t::rx_dma_min_timeout_ms.
 */
#define UART_BRIDGE_DEFAULT_RX_DMA_MIN_TIMEOUT_MS (50u)

/**
 * \brief Default upper clamp for \ref uart_bridge_config_t::rx_dma_max_timeout_ms.
 */
#define UART_BRIDGE_DEFAULT_RX_DMA_MAX_TIMEOUT_MS (200u)

/**
 * \brief Default formula for \ref uart_bridge_config_t::rx_drop_threshold.
 *
 * Marks the RX ring as drop-eligible once roughly a third of the slots are
 * pending, leaving headroom for the owner task to catch up before the
 * producer overruns.
 *
 * \param[in] count Total number of RX buffer slots in the owner's ring.
 */
#define UART_BRIDGE_DEFAULT_RX_DROP_THRESHOLD(count) ((count) / 3u)

/**
 * \brief Default polling period for \ref uart_bridge_config_t::tx_dma_check_finished_period_ms.
 *
 * Used by owners that wait on the UART line to drain after a TX DMA completion;
 * owners that do not perform TX (\c tx_buffer == \c NULL) should still set the
 * field to 0 explicitly.
 */
#define UART_BRIDGE_DEFAULT_TX_DMA_CHECK_FINISHED_PERIOD_MS (2u)

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/* Forward declaration so callbacks can reference the context. */
typedef struct uart_bridge_ctx uart_bridge_ctx_t;

/**
 * \brief Pass as \c flush to \ref uart_bridge_rx_sink_fn to request end-of-frame flushing.
 */
#define UART_BRIDGE_SINK_FLUSH (true)
/**
 * \brief Pass as \c flush when there is no end-of-frame; internal buffers need not be flushed.
 */
#define UART_BRIDGE_SINK_NO_FLUSH (false)

/**
 * \brief Pass as \c allow_drop to permit truncation when downstream is full.
 */
#define UART_BRIDGE_SINK_ALLOW_DROP (true)
/**
 * \brief Pass as \c allow_drop to require full delivery; the sink must stall or retry instead.
 */
#define UART_BRIDGE_SINK_NO_DROP (false)

/**
 * \brief Outcome of a sink callback (RX → USB / decoder).
 */
typedef enum uart_bridge_sink_result {
    UART_BRIDGE_SINK_OK = 0, /**< Buffer accepted; advance to the next one. */
    UART_BRIDGE_SINK_STALL,  /**< USB busy; abort the drain loop, retry on next notification. */
    UART_BRIDGE_SINK_RETRY,  /**< Decoder back-pressure; sleep 1 tick and retry the same buffer. */
} uart_bridge_sink_result_e;

/**
 * \brief Sink callback: forward RX payload from \a data into the owner's destination
 *        (CDC IN endpoint, ITM decoder, vendor bulk endpoint, etc.).
 *
 * Called from the owner task context.
 *
 * \param[in,out] ctx        Bridge context invoking the callback.
 * \param[in]     data       Pointer to the RX bytes inside the owner's static buffer pool.
 * \param[in]     len        Length of \a data in bytes.
 * \param[in]     flush      End-of-frame hint; the sink should flush any internal buffering.
 * \param[in]     allow_drop Allow truncation when downstream is full (back-pressure policy).
 * \return One of \ref uart_bridge_sink_result_e values.
 */
typedef uart_bridge_sink_result_e (*uart_bridge_rx_sink_fn)(
    uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop);

/**
 * \brief TX source callback: fill \a dst with up to \a cap bytes the owner wants to send.
 *
 * Called from the owner task context. Returning \c 0 means "nothing to send right now".
 *
 * \param[in,out] ctx Bridge context invoking the callback.
 * \param[out]    dst Destination buffer (typically \c cfg->tx_buffer).
 * \param[in]     cap Capacity of \a dst in bytes.
 * \return Number of bytes written into \a dst.
 */
typedef size_t (*uart_bridge_tx_source_fn)(uart_bridge_ctx_t *ctx, uint8_t *dst, size_t cap);

/**
 * \brief Optional notification callback fired from the UART ISR when RX activity is observed.
 *
 * Use to refresh activity LEDs. Must be ISR-safe.
 *
 * \param[in,out] ctx Bridge context.
 */
typedef void (*uart_bridge_on_rx_active_fn)(uart_bridge_ctx_t *ctx);

/**
 * \brief Cooperative ownership-release callback.
 *
 * The bridge invokes this on the current owner of a UART when another context wants to
 * claim it. Returning \c true means the owner has detached its hardware (UART IRQ handler
 * removed, DMA aborted, \c uart_deinit called) and the bridge may transfer ownership.
 *
 * Called from the requester's task context, never from an ISR.
 *
 * \c NULL in the config marks the owner as non-evictable (e.g. SWO).
 *
 * \param[in,out] ctx Bridge context being asked to release.
 * \return \c true if the owner agreed and detached its UART resources.
 */
typedef bool (*uart_bridge_on_release_request_fn)(uart_bridge_ctx_t *ctx);

/**
 * \brief Single GPIO pin → alternate-function mapping inside a \ref uart_bridge_binding_t.
 *
 * Populate \ref uart_bridge_binding_t::pins at index \ref UART_BRIDGE_BINDING_PIN_TX
 * and \ref UART_BRIDGE_BINDING_PIN_RX.  \ref gpio < 0 marks the slot as unused; the bridge
 * skips those entries during binding setup / teardown.  Owners mutate \c pins[].gpio at
 * init time from the board-specific pin map returned by \c platform_get_target_pins().
 */
typedef struct uart_bridge_pin_setup {
    int gpio;                 /**< GPIO number, or \c -1 when the slot is unused. */
    gpio_function_t function; /**< Pin function applied while the binding is active. */
} uart_bridge_pin_setup_t;

/**
 * \brief Description of a single UART hardware binding usable by a bridge channel.
 *
 * Owners declare one binding per UART instance they may switch to (main UART, TDI/TDO
 * UART, SWO UART, ...).  The bridge consults the binding when \ref uart_bridge_try_claim
 * switches contexts between UARTs and applies / reverts the GPIO alternate-function
 * map accordingly.  The bridge owns the UART-IRQ handler internally; owners do not
 * declare an ISR thunk and the IRQ line is resolved from \ref uart at compile time
 * via \ref UART_BRIDGE_UART_IRQ_NUM.
 */
typedef struct uart_bridge_binding {
    uart_inst_t *uart; /**< UART instance this binding targets. */
    /**
     * \brief TX/RX slots: indices \ref UART_BRIDGE_BINDING_PIN_TX and \ref UART_BRIDGE_BINDING_PIN_RX.
     */
    uart_bridge_pin_setup_t pins[UART_BRIDGE_MAX_PINS_PER_BINDING];
} uart_bridge_binding_t;

/**
 * \brief Static configuration of a bridge channel.
 *
 * The owner module keeps this as \c static \c const; the bridge stores a pointer to it
 * inside \ref uart_bridge_ctx_t and never modifies the structure.
 *
 * The \ref bindings array enumerates every UART instance this context may switch to;
 * each entry carries the pin map and IRQ-handler thunk used while the binding is
 * active.  \c bindings[0] is treated as the channel's "default" hardware (used only
 * by diagnostics — see \ref uart_bridge_init).
 */
typedef struct uart_bridge_config {
    uint8_t *rx_buffers_base;     /**< Base of the owner's \c [count][size] static RX buffer pool. */
    uint32_t rx_buffer_size;      /**< Size in bytes of one RX buffer slot. */
    uint32_t rx_buffer_count;     /**< Number of RX buffer slots in the pool. */
    uint8_t **rx_ctrl_block_info; /**< Owner's \c (count + 1) array used by the control DMA channel. */

    uint32_t rx_drop_threshold;         /**< Drop policy: drop buffers when this many are pending. */
    uint32_t rx_int_fifo_level;         /**< INT-mode FIFO trigger level (number of bytes drained per ISR). */
    uint32_t rx_dma_baudrate_threshold; /**< Choose DMA over INT when baudrate ≥ this value. */
    uint32_t rx_dma_min_timeout_ms;     /**< Lower clamp for the RX-idle timer period. */
    uint32_t rx_dma_max_timeout_ms;     /**< Upper clamp for the RX-idle timer period. */

    uint8_t *tx_buffer;                       /**< Owner's static TX DMA buffer; \c NULL disables TX on the channel. */
    uint32_t tx_buffer_size;                  /**< Capacity of \ref tx_buffer in bytes. */
    uint32_t tx_dma_check_finished_period_ms; /**< Polling period waiting for UART line to drain. */

    uint32_t notif_rx_available; /**< \c xTaskNotifyFromISR bit set when RX bytes are ready. */
    uint32_t notif_rx_timeout;   /**< Notification bit set on RX-idle timeout. */
    uint32_t notif_tx_complete;  /**< Notification bit set when TX DMA completes. */

    uart_bridge_rx_sink_fn rx_sink;                       /**< Required: RX → USB / decoder forwarder. */
    uart_bridge_tx_source_fn tx_source;                   /**< Optional: TX byte producer; \c NULL when TX disabled. */
    uart_bridge_on_rx_active_fn on_rx_active;             /**< Optional: ISR-safe LED hook. */
    uart_bridge_on_release_request_fn on_release_request; /**< Optional: cooperative-eviction hook. */

    /**
     * Array of UART hardware bindings this context may switch to.
     *
     * The bridge picks an entry by matching \c new_uart in \ref uart_bridge_try_claim
     * and uses it to drive GPIO alternate-function selection.  The UART IRQ line is
     * derived from \c binding->uart through \ref UART_BRIDGE_UART_IRQ_NUM and dispatched
     * by the bridge's central per-UART thunk; owners no longer declare per-binding
     * ISR thunks.
     *
     * Owners are free to mutate \c pins[].gpio at init time (after the platform pin
     * map is available); the bridge never writes to this array.
     */
    const uart_bridge_binding_t *bindings;
    uint32_t bindings_count; /**< Number of valid entries in \ref bindings. */

    const char *timer_name; /**< Name used in \c xTimerCreate for diagnostics. */
    void *user_ctx;         /**< Owner-defined pointer; bridge does not interpret it. */
} uart_bridge_config_t;

/**
 * \brief Mutable runtime state of a bridge channel.
 *
 * Owner provides storage; bridge initialises and maintains the fields. Size is on the
 * order of tens of bytes; no heap allocation is performed.
 */
struct uart_bridge_ctx {
    const uart_bridge_config_t *cfg; /**< Pointer to the owner's static config. */

    uart_inst_t *uart; /**< Currently bound UART instance (\c NULL when unbound). */

    int rx_dma_channel;      /**< RX-data DMA channel; \ref DMA_EX_CHANNEL_UNCLAIMED if unclaimed. */
    int rx_dma_ctrl_channel; /**< RX control-list DMA channel; \ref DMA_EX_CHANNEL_UNCLAIMED if unclaimed. */
    int tx_dma_channel;      /**< TX DMA channel; \ref DMA_EX_CHANNEL_UNCLAIMED when TX disabled. */

    bool rx_use_dma;                     /**< Selected RX path: \c true = DMA, \c false = INT. */
    bool rx_ongoing;                     /**< Whether an RX session is currently active. */
    uint32_t rx_int_buf_pos;             /**< Wrap pointer into the flat RX buffer pool for INT-mode. */
    uint32_t rx_dma_buffer_full_mask;    /**< One bit per RX slot: set when DMA filled it. */
    uint32_t rx_dma_current_buffer;      /**< Index of the buffer currently being filled by DMA. */
    uint32_t rx_dma_next_buffer_to_send; /**< Index of the next slot to drain in the task. */

    bool tx_ongoing;      /**< Whether a TX DMA transfer is in flight. */
    bool tx_dma_finished; /**< Set when DMA completion was observed; UART may still be draining. */

    TimerHandle_t rx_timeout_timer; /**< Per-context RX-idle timer; ID is the context pointer. */
    TaskHandle_t owner_task;        /**< Task to receive notifications. */
};

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief One-time global initialisation of the bridge subsystem.
 *
 * Creates the bridge structural-operations mutex via \c xSemaphoreCreateMutex.
 * Must be called exactly once, from a single-threaded context before the scheduler
 * starts or before any task that invokes the bridge API is allowed to run.
 * Must not be called from inside a critical section or from an ISR.
 */
void uart_bridge_common_init(void);

/**
 * \brief One-time initialisation of a bridge channel.
 *
 * Stores \a cfg and \a owner_task in \a ctx, leaves \c ctx->uart unbound (no UART
 * hardware is driven before the first \ref uart_bridge_try_claim), claims DMA
 * channels (RX, RX control, and TX when \c cfg->tx_buffer is non-\c NULL), populates
 * \c cfg->rx_ctrl_block_info with pointers into \c cfg->rx_buffers_base, creates the
 * per-context RX-idle timer, and on first call installs the exclusive
 * \c DMA_IRQ_0 dispatcher.
 *
 * Does NOT touch the ownership table; call \ref uart_bridge_try_claim before driving
 * the UART hardware.  The first claim also performs binding-driven GPIO and UART-IRQ
 * installation; \ref uart_bridge_init only validates \c cfg->bindings.
 *
 * \param[out] ctx        Caller-provided context storage.
 * \param[in]  cfg        Owner's static config (must outlive \a ctx).
 * \param[in]  owner_task Task to receive notifications via \c xTaskNotifyFromISR.
 */
void uart_bridge_init(uart_bridge_ctx_t *ctx, const uart_bridge_config_t *cfg, TaskHandle_t owner_task);

/**
 * \brief (Re)configure the currently bound UART for a given line coding.
 *
 * Disables previous DMA / IRQs, applies \c uart_init / \c uart_set_format, decides
 * between INT and DMA RX paths using \c cfg->rx_dma_baudrate_threshold, recomputes the
 * RX-idle timer period, and reconfigures the TX DMA channel (when TX is enabled).
 *
 * \param[in,out] ctx       Bridge context bound to a UART (call \ref uart_bridge_try_claim first).
 * \param[in]     baudrate  Bits per second.
 * \param[in]     data_bits Word length, 5–8.
 * \param[in]     stop_bits 1 or 2.
 * \param[in]     parity    \c uart_parity_t value.
 */
void uart_bridge_configure_uart(
    uart_bridge_ctx_t *ctx, uint32_t baudrate, uint8_t data_bits, uint8_t stop_bits, uart_parity_t parity);

/**
 * \brief Drain the INT-mode RX buffer into the sink and re-arm the RX FIFO IRQ.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_rx_int_process(uart_bridge_ctx_t *ctx);

/**
 * \brief Finalise an INT-mode RX session: flush remaining FIFO bytes, sink with \c flush=true,
 *        re-arm the RX timeout IRQ.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_rx_int_finish(uart_bridge_ctx_t *ctx);

/**
 * \brief Drain DMA-filled RX buffer slots into the sink.
 *
 * Called from the owner task on \c notif_rx_available. Loops while the next-to-send slot
 * has its full-bit set and the sink keeps accepting; honours back-pressure and
 * decoder-stall via \ref UART_BRIDGE_SINK_STALL / \ref UART_BRIDGE_SINK_RETRY.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_rx_dma_process_buffers(uart_bridge_ctx_t *ctx);

/**
 * \brief Finalise a DMA-mode RX session: drain in-flight buffer, restart control DMA.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_rx_dma_finish_receiving(uart_bridge_ctx_t *ctx);

/**
 * \brief Pull a chunk from \c cfg->tx_source and start a TX DMA transfer.
 *
 * No-op when TX is disabled (\c cfg->tx_buffer == NULL). When the source returns 0 and a
 * transfer is still in flight, \c ctx->tx_dma_finished is raised so the owner task can
 * poll \ref uart_bridge_tx_dma_check_finished.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_tx_dma_send(uart_bridge_ctx_t *ctx);

/**
 * \brief Check whether the UART line has fully drained after a TX DMA completion.
 *
 * \param[in,out] ctx Bridge context.
 * \return \c true if the UART transmitter is idle (or TX is disabled).
 */
bool uart_bridge_tx_dma_check_finished(uart_bridge_ctx_t *ctx);

/**
 * \brief Attempt to acquire \a new_uart for \a ctx.
 *
 * The IRQ line associated with \a new_uart is looked up in \c ctx->cfg->bindings, so
 * the caller no longer passes it explicitly; if \a new_uart is not declared in any
 * binding the call fails (asserted in debug builds).
 *
 * Behaviour:
 *  - free in the table: claim unconditionally;
 *  - already owned by \a ctx: no-op success;
 *  - owned by another context and \a force == \ref UART_BRIDGE_CLAIM_FORCE:
 *    invoke the current owner's \c on_release_request callback; succeed if the
 *    callback is non-NULL and returns \c true;
 *  - owned by another context and \a force == \ref UART_BRIDGE_CLAIM_NO_FORCE:
 *    return \c false immediately without touching the current owner.
 *
 * On a UART transition (the context previously owned a different UART, or just took
 * an empty slot), the bridge runs binding-driven hardware setup atomically with the
 * ownership update:
 *  - reverts the GPIO map of the previously active binding (when applicable) and
 *    disables the corresponding UART IRQ line in the NVIC;
 *  - applies the new binding's \c pins[] alternate-function map;
 *  - publishes \a ctx into the bridge's per-UART dispatcher slot and enables the
 *    UART IRQ line in the NVIC.  The bridge's central per-UART ISR thunk is the
 *    sole NVIC handler for both UART IRQ lines and is installed lazily on the
 *    first claim.
 *
 * On success, \a ctx becomes the sole owner of \a new_uart in the table; any previous
 * UART claimed by \a ctx is released from the table and its hardware is torn down by
 * the bridge.
 *
 * \param[in,out] ctx       Bridge context.
 * \param[in]     new_uart  UART instance to claim; must be declared in \c ctx->cfg->bindings.
 * \param[in]     force     \ref UART_BRIDGE_CLAIM_FORCE or \ref UART_BRIDGE_CLAIM_NO_FORCE.
 * \return \c true on success, \c false otherwise.
 */
bool uart_bridge_try_claim(uart_bridge_ctx_t *ctx, uart_inst_t *new_uart, bool force);

/**
 * \brief Release any UART entries owned by \a ctx in the bridge table.
 *
 * Does not invoke any callbacks and does not touch the UART hardware; the caller is
 * responsible for DMA abort and \c uart_deinit.  The bridge's central UART dispatcher
 * is not retracted here either — call \ref uart_bridge_deinit_uart first when the
 * NVIC line for \c ctx->uart must also be masked.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_release(uart_bridge_ctx_t *ctx);

/**
 * \brief Look up the current owner of \a uart in the bridge table.
 *
 * \param[in] uart UART instance.
 * \return Owning context, or \c NULL when the UART is free.
 */
uart_bridge_ctx_t *uart_bridge_get_owner(uart_inst_t *uart);

/**
 * \brief Symmetric teardown of the UART hardware bound to \a ctx.
 *
 * Symmetric counterpart to \ref uart_bridge_configure_uart.  Suitable for use in
 * \c on_release_request callbacks and \c swo_deinit-style explicit teardowns where the
 * owner wants to temporarily release the UART while keeping its DMA channels and timer
 * alive for a future re-claim.
 *
 * Sequence:
 *  1. \c xTimerStop (non-blocking).
 *  2. For each valid DMA channel: disable IRQ0, abort, acknowledge IRQ0.
 *  3. If \c ctx->uart is non-\c NULL:
 *     - \c uart_ex_set_dma_req_enabled(false, false)
 *     - \c uart_ex_set_rx_and_timeout_irq_enabled(false, false)
 *     - \c uart_ex_clear_rx_and_rx_timeout_irq_flags
 *     - mask the binding's UART IRQ line in the NVIC and retract the bridge's
 *       per-UART dispatcher slot for \c ctx (the central ISR handler itself
 *       stays installed for the lifetime of the bridge)
 *     - return the binding's GPIO pins to \c GPIO_FUNC_SIO
 *     - \c uart_deinit
 *  4. Reset all runtime-state fields (rx/tx ongoing, buffer indices, masks).
 *
 * Does \b not release the ownership table entry and does \b not unclaim DMA channels.
 * Call \ref uart_bridge_release after this when ownership should also be surrendered.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_deinit_uart(uart_bridge_ctx_t *ctx);

/**
 * \brief Full teardown of a bridge channel; symmetric to \ref uart_bridge_init.
 *
 * Calls \ref uart_bridge_deinit_uart, then \ref uart_bridge_release, deletes the
 * RX-idle timer, unclaims all DMA channels, and removes the context from the
 * \c DMA_IRQ_0 dispatcher (uninstalling the exclusive handler when no other
 * context remains registered).
 *
 * Intended for system-shutdown or module-destroy scenarios; not used in normal
 * operation where only the UART hardware needs to be released temporarily.
 *
 * \param[in,out] ctx Bridge context.
 */
void uart_bridge_deinit(uart_bridge_ctx_t *ctx);

#endif /* MIOLINK_UART_BRIDGE_H */
