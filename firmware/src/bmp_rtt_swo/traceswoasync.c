/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
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

#define TRACESWO_UART_RX_INT_FIFO_LEVEL \
    (UART_BRIDGE_DEFAULT_RX_INT_FIFO_LEVEL) /**< UART RX FIFO trigger level used in IRQ mode. */

#define TRACESWO_RX_DMA_TOTAL_BUFFERS_SIZE (16 * 1024) /**< Total RX DMA staging area, in bytes. */
#define TRACESWO_RX_DMA_NUMBER_OF_BUFFERS  (32)        /**< Number of DMA ring buffers. */
#define TRACESWO_RX_DMA_BUFFER_SIZE \
    (TRACESWO_RX_DMA_TOTAL_BUFFERS_SIZE / TRACESWO_RX_DMA_NUMBER_OF_BUFFERS) /**< Single DMA buffer size, in bytes. */

#define TRACESWO_RX_DMA_DROP_BUFFER_THRESHOLD \
    (UART_BRIDGE_DEFAULT_RX_DROP_THRESHOLD(TRACESWO_RX_DMA_NUMBER_OF_BUFFERS)) /**< Drop threshold. */

#if (TRACESWO_RX_DMA_NUMBER_OF_BUFFERS < 4)
#error "TRACESWO_RX_DMA_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define TRACESWO_RX_DMA_BAUDRATE_THRESHOLD \
    (UART_BRIDGE_DEFAULT_RX_DMA_BAUDRATE_THRESHOLD) /**< Below this baud, use IRQ path instead of DMA. */
/** Lower bound on DMA RX timeout, in ms. */
#define TRACESWO_RX_DMA_MIN_TIMEOUT (UART_BRIDGE_DEFAULT_RX_DMA_MIN_TIMEOUT_MS)
/** Upper bound on DMA RX timeout, in ms. */
#define TRACESWO_RX_DMA_MAX_TIMEOUT (UART_BRIDGE_DEFAULT_RX_DMA_MAX_TIMEOUT_MS)
#define TRACESWO_TASK_NOTIFY_WAIT_PERIOD (portMAX_DELAY) /**< Notification wait period for the trace task. */

#define TRACESWO_VENDOR_INTERFACE (0) /**< TinyUSB vendor interface index used to stream SWO bytes. */

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Slot indices into \ref s_trace_bindings.
 *
 * \c TRACE_BINDING_COUNT trails the named entries and doubles as the array size
 * and as \ref uart_bridge_config_t::bindings_count.
 */
typedef enum trace_binding {
    TRACE_BINDING_IDX_SWO = 0, /**< SWO UART binding. */
    TRACE_BINDING_COUNT,       /**< Number of entries in \ref s_trace_bindings. */
} trace_binding_e;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief Ring of DMA RX buffers receiving raw SWO bytes from the UART.
 */
static uint8_t uart_rx_buf[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS][TRACESWO_RX_DMA_BUFFER_SIZE] = {0};

/**
 * \brief Control-block list for the chained RX DMA.
 *
 * Alignment matches the ring-wrap window (count * sizeof(uint32_t)) used by
 * \c channel_config_set_ring inside the bridge.
 */
static uint8_t *uart_dma_rx_ctrl_block_info[TRACESWO_RX_DMA_NUMBER_OF_BUFFERS + 1]
    __attribute__((aligned(TRACESWO_RX_DMA_NUMBER_OF_BUFFERS * sizeof(uint32_t)))) = {0};

static uart_bridge_ctx_t s_trace_ctx = {0}; /**< UART-bridge context for the SWO RX path. */

static bool traceswo_decoding = false;    /**< When \c true, decode ITM stream instead of raw forwarding. */
static TaskHandle_t traceswo_task = NULL; /**< Handle of the SWO worker task. */

/**
 * \brief Active SWO mode after the last \c swo_init / \c swo_deinit.
 */
swo_coding_e swo_current_mode = swo_none;

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Update the serial-activity LED based on the bridge \c rx_ongoing flag.
 */
static void traceswo_update_led(void);

/**
 * \brief Push raw SWO bytes into the TinyUSB vendor interface, optionally dropping on back-pressure.
 *
 * \param[in] data              Buffer of SWO bytes to forward.
 * \param[in] len               Number of bytes in \a data.
 * \param[in] flush             Force USB FIFO flush after writing.
 * \param[in] allow_drop_buffer Allow partial transmission when USB write space is insufficient.
 * \return \c true if the call was accepted (full or partial); \c false on stall.
 */
static bool traceswo_send_to_usb(uint8_t *data, size_t len, bool flush, bool allow_drop_buffer);

/**
 * \brief UART-bridge sink callback: either decode ITM or forward raw bytes to USB.
 *
 * \param[in,out] ctx        Bridge context (unused).
 * \param[in]     data       Bytes from the UART receiver.
 * \param[in]     len        Number of bytes in \a data.
 * \param[in]     flush      Hint to flush downstream now.
 * \param[in]     allow_drop Allow dropping bytes when downstream is full.
 * \return \c UART_BRIDGE_SINK_OK on success; \c UART_BRIDGE_SINK_RETRY or \c UART_BRIDGE_SINK_STALL on back-pressure.
 */
static uart_bridge_sink_result_e trace_sink(
    uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop);

/**
 * \brief UART-bridge "RX became active" hook used to refresh the activity LED.
 *
 * \param[in,out] ctx Bridge context (unused).
 */
static void trace_on_rx_active(uart_bridge_ctx_t *ctx);

/**
 * \brief SWO FreeRTOS task body: pumps the UART bridge in response to task notifications.
 *
 * \param[in] params Unused FreeRTOS parameter.
 */
static void traceswo_thread(void *params);

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * Single hardware binding for the SWO UART.  Pin numbers are filled in
 * \ref traceswo_task_init from \c platform_get_target_pins(); the bridge treats this
 * array as read-only after that.  \c tdi is intentionally forced back to \c SIO when
 * the binding becomes active so the pin does not drive while \c uart0 is in SWO mode.
 *
 * The UART NVIC handler is owned by the bridge and dispatched from its central
 * per-UART ISR thunk; this binding only describes the GPIO map and the target UART.
 */
static uart_bridge_binding_t s_trace_bindings[TRACE_BINDING_COUNT] = {
    [TRACE_BINDING_IDX_SWO] =
        {
            .uart = TRACESWO_UART,
            .pins =
                {
                    [UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
                    [UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_SIO},
                },
        },
};

/**
 * \brief UART-bridge configuration for the SWO trace channel.
 *
 * TX is unused (SWO is RX-only), so the TX buffer fields are zeroed. \c on_release_request is
 * \c NULL, making the binding non-evictable: a cooperative claim from another channel must
 * fail until \c swo_deinit releases the UART explicitly.
 */
static const uart_bridge_config_t s_trace_cfg = {
    .rx_buffers_base = (uint8_t *)uart_rx_buf,
    .rx_buffer_size = TRACESWO_RX_DMA_BUFFER_SIZE,
    .rx_buffer_count = TRACESWO_RX_DMA_NUMBER_OF_BUFFERS,
    .rx_ctrl_block_info = uart_dma_rx_ctrl_block_info,
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

    if ((result) && (flush)) {
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

    /* Claim the UART.  FORCE cooperatively evicts \c target_serial when it currently
     * holds \c TRACESWO_UART (SWO has priority).  The bridge applies the matching
     * binding's GPIO functions and installs the shared UART-IRQ handler atomically
     * with the ownership update, so no separate \c gpio_set_function / \c irq_*
     * sequence is required here. */
    if (uart_bridge_try_claim(&s_trace_ctx, TRACESWO_UART, UART_BRIDGE_CLAIM_FORCE) == false) {
        return;
    }

    uart_bridge_configure_uart(&s_trace_ctx, baudrate, 8, 1, UART_PARITY_NONE);

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
        TRACESWO_TASK_PRIORITY, TRACESWO_TASK_CORE_AFFINITY, &traceswo_task);
#else
    const BaseType_t result = xTaskCreate(
        traceswo_thread, "target_trace", TRACESWO_TASK_STACK_SIZE, NULL, TRACESWO_TASK_PRIORITY, &traceswo_task);
#endif
    assert(result == pdPASS);

    /* Initialise the bridge here, after the task handle is known but before the
     * scheduler is resumed in \c main.  This guarantees \c s_trace_ctx is fully
     * set up before \c swo_init can reach \c uart_bridge_try_claim from another task. */
    uart_bridge_init(&s_trace_ctx, &s_trace_cfg, traceswo_task);
}

uint32_t swo_get_baudrate(void)
{
    if (swo_current_mode == swo_nrz_uart) {
        return uart_ex_get_baudrate(TRACESWO_UART);
    }
    return 0;
}
