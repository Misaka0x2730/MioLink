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

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

#include "uart_bridge.h"
#include "usb_cdc.h"
#include "target_serial.h"

#if defined(ENABLE_SEGGER_RTT)
#include "SEGGER_RTT.h"
#endif

#if defined(ENABLE_RTT)
#include "rtt.h"
#endif

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define TARGET_SERIAL_UART_RX_INT_FIFO_LEVEL \
    (UART_BRIDGE_DEFAULT_RX_INT_FIFO_LEVEL) /**< UART RX FIFO trigger level used in IRQ mode. */

#define TARGET_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE (16 * 1024) /**< Total RX DMA staging area, in bytes. */
#define TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS  (32)        /**< Number of DMA RX ring buffers. */
#define TARGET_SERIAL_UART_DMA_RX_BUFFER_SIZE       \
    (TARGET_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE / \
        TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS) /**< Single RX buffer size. */

/**
 * \brief Drop-buffer threshold for the target-serial RX DMA ring.
 */
#define TARGET_SERIAL_UART_DMA_RX_DROP_BUFFER_THRESHOLD \
    (UART_BRIDGE_DEFAULT_RX_DROP_THRESHOLD(TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS))

#if (TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define TARGET_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD \
    (UART_BRIDGE_DEFAULT_RX_DMA_BAUDRATE_THRESHOLD) /**< Below this baud, use IRQ RX path instead of DMA. */
/** Minimum DMA RX idle timeout, ms. */
#define TARGET_SERIAL_UART_DMA_RX_MIN_TIMEOUT (UART_BRIDGE_DEFAULT_RX_DMA_MIN_TIMEOUT_MS)
/** Maximum DMA RX idle timeout, ms. */
#define TARGET_SERIAL_UART_DMA_RX_MAX_TIMEOUT (UART_BRIDGE_DEFAULT_RX_DMA_MAX_TIMEOUT_MS)

#define TARGET_SERIAL_UART_DMA_TX_BUFFER_SIZE (256) /**< TX DMA staging buffer size, bytes. */
#define TARGET_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD_MS \
    (UART_BRIDGE_DEFAULT_TX_DMA_CHECK_FINISHED_PERIOD_MS) /**< Polling period for TX-finished check, ms. */

#define TARGET_SERIAL_TASK_NOTIFY_WAIT_PERIOD (portMAX_DELAY) /**< FreeRTOS task notify wait period. */

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Slot indices into \c s_serial_bindings.
 *
 * \c SERIAL_BINDING_COUNT trails the named entries and doubles as the array size
 * and as \ref uart_bridge_config_t::bindings_count.
 */
typedef enum serial_binding {
    SERIAL_BINDING_MAIN = 0, /**< Main USB-serial UART binding. */
    SERIAL_BINDING_TDI_TDO,  /**< TDI/TDO UART binding. */
    SERIAL_BINDING_COUNT,    /**< Number of entries in \c s_serial_bindings. */
} serial_binding_e;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief Ring of DMA RX staging buffers receiving target UART bytes.
 */
static uint8_t uart_rx_buf[TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS][TARGET_SERIAL_UART_DMA_RX_BUFFER_SIZE] = {0};
static uint8_t uart_tx_dma_buf[TARGET_SERIAL_UART_DMA_TX_BUFFER_SIZE] = {0}; /**< Single DMA TX staging buffer. */

/**
 * \brief Control-block list for the chained RX DMA.
 *
 * Alignment matches the ring-wrap window (count * sizeof(uint32_t)) used by
 * \c channel_config_set_ring inside the bridge.
 */
static uint8_t *uart_dma_rx_ctrl_block_info[TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS + 1]
    __attribute__((aligned(TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS * sizeof(uint32_t)))) = {0};

static uart_bridge_ctx_t s_serial_ctx = {0}; /**< Bridge context for the target-serial UART. */

/**
 * \brief \c true when target serial is routed through TDI/TDO instead of MAIN.
 *
 * Written from the GDB task (core 1) by \ref target_serial_use_uart_on_tdi_tdo and read by
 * \c target_serial_thread on core 0; \c volatile forces a fresh load on the consumer side.
 */
static volatile bool use_uart_on_tdi_tdo = false;

/**
 * \brief \c true while the JTAG TAP holds TDI/TDO; suppresses the TDI/TDO UART binding.
 *
 * Asserted by \ref target_serial_tap_acquire_tdi_tdo from \c jtagtap_init (core 1) and
 * cleared by \ref target_serial_tap_release_tdi_tdo from \c swdptap_init.  Read by
 * \c target_serial_thread (core 0) when computing the desired UART; while set, the bridge
 * binds \c TARGET_SERIAL_UART_MAIN regardless of \ref use_uart_on_tdi_tdo so the JTAG PIO
 * has exclusive ownership of the TDI/TDO GPIOs.
 */
static volatile bool tdi_tdo_held_by_tap = false;

static TaskHandle_t usb_uart_task = NULL; /**< Handle of the target-serial worker FreeRTOS task. */

#if defined(ENABLE_RTT)
extern void rtt_serial_receive_callback(void);
#endif

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief UART-bridge sink callback: forward target-UART RX bytes to the USB CDC.
 *
 * \param[in,out] ctx        Bridge context (unused).
 * \param[in]     data       Bytes received from the target UART.
 * \param[in]     len        Number of bytes in \a data.
 * \param[in]     flush      Hint to flush the USB FIFO after the call.
 * \param[in]     allow_drop Allow dropping bytes when USB stalls.
 * \return \c UART_BRIDGE_SINK_OK on accept, \c UART_BRIDGE_SINK_STALL otherwise.
 */
static uart_bridge_sink_result_e serial_sink(
    uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop);

/**
 * \brief UART-bridge TX source callback: pull bytes from the USB CDC into the bridge TX buffer.
 *
 * \param[in,out] ctx Bridge context (unused).
 * \param[out]    dst Destination buffer in the bridge.
 * \param[in]     cap Capacity of \a dst in bytes.
 * \return Number of bytes consumed from the USB CDC.
 */
static size_t serial_tx_source(uart_bridge_ctx_t *ctx, uint8_t *dst, size_t cap);

/**
 * \brief "RX became active" hook used to refresh the activity LED.
 *
 * \param[in,out] ctx Bridge context (unused).
 */
static void serial_on_rx_active(uart_bridge_ctx_t *ctx);

/**
 * \brief Cooperative-release hook: invoked by the bridge when another module (e.g. SWO) wants the UART.
 *
 * \param[in,out] ctx Bridge context.
 * \return Always \c true (we relinquish the UART unconditionally).
 */
static bool serial_on_release_request(uart_bridge_ctx_t *ctx);

/**
 * \brief Apply a new CDC line coding to the bound UART, switching binding (MAIN ↔ TDI/TDO) if needed.
 *
 * \param[in] line_coding USB CDC line-coding descriptor from the host.
 */
static void serial_update_config(cdc_line_coding_t *line_coding);

/**
 * \brief Target-serial FreeRTOS task body: drives the UART bridge and reacts to line-state changes.
 *
 * \param[in] params Unused FreeRTOS parameter.
 */
static void target_serial_thread(void *params);

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * Hardware bindings consumed by the bridge.  Pin numbers are filled at runtime in
 * \ref target_serial_init from \c platform_get_target_pins(); the bridge treats this
 * array as read-only afterwards.  Both bindings share the bridge's central UART
 * dispatcher; the NVIC handler is owned by the bridge and resolves the active
 * context from the per-UART dispatcher slot updated on every claim.
 */
static uart_bridge_binding_t s_serial_bindings[SERIAL_BINDING_COUNT] = {
    [SERIAL_BINDING_MAIN] =
        {
            .uart = TARGET_SERIAL_UART_MAIN,
            .pins =
                {
                    [UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
                    [UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_UART},
                },
        },
    [SERIAL_BINDING_TDI_TDO] =
        {
            .uart = TARGET_SERIAL_UART_TDI_TDO,
            .pins =
                {
                    [UART_BRIDGE_BINDING_PIN_TX] = {-1, GPIO_FUNC_UART},
                    [UART_BRIDGE_BINDING_PIN_RX] = {-1, GPIO_FUNC_UART},
                },
        },
};

/**
 * \brief UART-bridge configuration for the target-serial CDC channel.
 *
 * Wires the shared DMA-driven RX/TX buffers, drop thresholds, baud-rate-based DMA threshold,
 * USB-CDC notification bits, and binding table to the bridge core. The bridge treats this
 * struct as immutable; per-claim state lives in the bridge's own context.
 */
static const uart_bridge_config_t s_serial_cfg = {
    .rx_buffers_base = (uint8_t *)uart_rx_buf,
    .rx_buffer_size = TARGET_SERIAL_UART_DMA_RX_BUFFER_SIZE,
    .rx_buffer_count = TARGET_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS,
    .rx_ctrl_block_info = uart_dma_rx_ctrl_block_info,
    .rx_drop_threshold = TARGET_SERIAL_UART_DMA_RX_DROP_BUFFER_THRESHOLD,
    .rx_int_fifo_level = TARGET_SERIAL_UART_RX_INT_FIFO_LEVEL,
    .rx_dma_baudrate_threshold = TARGET_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD,
    .rx_dma_min_timeout_ms = TARGET_SERIAL_UART_DMA_RX_MIN_TIMEOUT,
    .rx_dma_max_timeout_ms = TARGET_SERIAL_UART_DMA_RX_MAX_TIMEOUT,
    .tx_buffer = uart_tx_dma_buf,
    .tx_buffer_size = sizeof(uart_tx_dma_buf),
    .tx_dma_check_finished_period_ms = TARGET_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD_MS,
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

static uart_bridge_sink_result_e serial_sink(
    uart_bridge_ctx_t *ctx, uint8_t *data, size_t len, bool flush, bool allow_drop)
{
    (void)ctx;
    if (target_serial_send_to_usb(data, len, flush, allow_drop)) {
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
    target_serial_update_led();
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
    case CDC_LINE_CODING_STOP_BITS_1:
    case CDC_LINE_CODING_STOP_BITS_1_5:
        stop_bits = 1;
        break;
    case CDC_LINE_CODING_STOP_BITS_2:
        /* Fall through. */
    default:
        break;
    }

    uart_parity_t parity = UART_PARITY_NONE;
    switch (line_coding->parity) {
    case CDC_LINE_CODING_PARITY_ODD:
        parity = UART_PARITY_ODD;
        break;
    case CDC_LINE_CODING_PARITY_EVEN:
        parity = UART_PARITY_EVEN;
        break;
    case CDC_LINE_CODING_PARITY_NONE:
        /* Fall through. */
    default:
        break;
    }

    uint8_t data_bits = 8;
    if (line_coding->data_bits <= 8) {
        data_bits = line_coding->data_bits;
    }

    uart_inst_t *const desired_uart =
        ((use_uart_on_tdi_tdo) && (!tdi_tdo_held_by_tap)) ? TARGET_SERIAL_UART_TDI_TDO : TARGET_SERIAL_UART_MAIN;

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

    uint32_t notification_value = 0;
    uint32_t wait_time = TARGET_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

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
#if defined(ENABLE_RTT)
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
                wait_time = TARGET_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

                s_serial_ctx.tx_ongoing = false;
                s_serial_ctx.tx_dma_finished = false;
            } else {
                wait_time = pdMS_TO_TICKS(TARGET_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD_MS);
            }
        }

        uart_inst_t *const desired_uart =
            ((use_uart_on_tdi_tdo) && (!tdi_tdo_held_by_tap)) ? TARGET_SERIAL_UART_TDI_TDO : TARGET_SERIAL_UART_MAIN;
        if (s_serial_ctx.uart != desired_uart) {
            cdc_line_coding_t line_coding = {0};
            tud_cdc_n_get_line_coding(USB_CDC_TARGET_SERIAL, &line_coding);
            serial_update_config(&line_coding);
        }

        target_serial_update_led();
    }
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

bool target_serial_get_dtr(void)
{
    return (tud_cdc_n_get_line_state(USB_CDC_TARGET_SERIAL) & 0x01) != 0;
}

void target_serial_update_led(void)
{
    if (tud_cdc_n_connected(USB_CDC_TARGET_SERIAL) == false) {
        platform_set_serial_state(false);
    } else {
        platform_set_serial_state(s_serial_ctx.rx_ongoing || s_serial_ctx.tx_ongoing);
    }
}

uint16_t target_serial_get_available(void)
{
    return tud_cdc_n_write_available(USB_CDC_TARGET_SERIAL);
}

uint32_t target_serial_read(uint8_t *data, const uint32_t buffer_size)
{
    return tud_cdc_n_read(USB_CDC_TARGET_SERIAL, data, buffer_size);
}

bool target_serial_send_to_usb(uint8_t *data, const size_t len, bool flush, const bool allow_drop_buffer)
{
    bool result = false;
    const uint32_t write_available = tud_cdc_n_write_available(USB_CDC_TARGET_SERIAL);

    if (target_serial_get_dtr() == USB_CDC_DTR_DEASSERTED) {
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

    if ((result) && (flush)) {
        tud_cdc_n_write_flush(USB_CDC_TARGET_SERIAL);
    }

    return result;
}

void target_serial_use_uart_on_tdi_tdo(const bool new_state)
{
    use_uart_on_tdi_tdo = new_state;
    if (usb_uart_task != NULL) {
        xTaskNotify(usb_uart_task, USB_CDC_NOTIF_DUMMY, eSetBits);
    }
}

bool target_serial_uart_on_tdi_tdo_is_used(void)
{
    return use_uart_on_tdi_tdo;
}

void target_serial_tap_acquire_tdi_tdo(void)
{
    /* Assert the lockout first so any concurrent serial-task iteration that re-evaluates
     * desired_uart after this point will resolve to MAIN, preventing it from re-claiming
     * TDI/TDO once we drop our hold below. */
    tdi_tdo_held_by_tap = true;

    /* Detach UART hardware + return TDI/TDO GPIOs to SIO only if we still hold them.
     * If the serial task has already swapped us to MAIN, the TDI/TDO slot is unowned and
     * deinit would mistakenly tear down MAIN.  uart_bridge_get_owner takes the bridge
     * mutex, which also acts as a cross-core memory barrier publishing the lockout write
     * above before any subsequent claim by the serial task. */
    if (uart_bridge_get_owner(TARGET_SERIAL_UART_TDI_TDO) == &s_serial_ctx) {
        uart_bridge_deinit_uart(&s_serial_ctx);
    }

    /* Wake the serial task so it picks up MAIN without waiting for the polling timeout. */
    if (usb_uart_task != NULL) {
        xTaskNotify(usb_uart_task, USB_CDC_NOTIF_DUMMY, eSetBits);
    }
}

void target_serial_tap_release_tdi_tdo(void)
{
    tdi_tdo_held_by_tap = false;

    /* If the user previously enabled UART-on-TDI/TDO the polling loop will see the
     * lockout drop and rebind TDI/TDO; a notify just shortens the latency. */
    if (usb_uart_task != NULL) {
        xTaskNotify(usb_uart_task, USB_CDC_NOTIF_DUMMY, eSetBits);
    }
}

void target_serial_init(void)
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
    const BaseType_t result = xTaskCreateAffinitySet(target_serial_thread, "target_uart", TARGET_SERIAL_TASK_STACK_SIZE,
        NULL, TARGET_SERIAL_TASK_PRIORITY, TARGET_SERIAL_TASK_CORE_AFFINITY, &usb_uart_task);
#else
    const BaseType_t result = xTaskCreate(target_serial_thread, "target_uart", TARGET_SERIAL_TASK_STACK_SIZE, NULL,
        TARGET_SERIAL_TASK_PRIORITY, &usb_uart_task);
#endif

    assert(result == pdPASS);

    /* Initialise the bridge here, after the task handle is known but before the
     * scheduler is resumed in \c main.  This guarantees \c s_serial_ctx is fully
     * set up before any task can call into \c uart_bridge_try_claim against it. */
    uart_bridge_init(&s_serial_ctx, &s_serial_cfg, usb_uart_task);

    usb_cdc_register_listener(
        USB_CDC_TARGET_SERIAL, usb_uart_task, USB_CDC_NOTIF_USB_RX_AVAILABLE | USB_CDC_NOTIF_LINE_CODING_UPDATE);
}

/**
 * \brief Black Magic debug-stdout sink: forward bytes onto the target-serial CDC with flush.
 *
 * \param[in] data Bytes to send.
 * \param[in] len  Length of \a data in bytes.
 */
void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{
    target_serial_send_to_usb((uint8_t *)data, len, true, true);
}

#if ENABLE_DEBUG == 1
/**
 * \brief Black Magic DEBUG sink: forward debug bytes onto the target-serial CDC.
 *
 * \param[in] buf Bytes to send.
 * \param[in] len Length of \a buf.
 * \return \a len on success; \c 0 on failure.
 */
size_t debug_serial_debug_write(const char *buf, const size_t len)
{
    return target_serial_send_to_usb((uint8_t *)buf, len, true, true) ? len : 0;
}

/**
 * \brief Newlib \c _write hook routing stdout to either the BMP debug stream or SEGGER RTT.
 *
 * \param[in] file Ignored.
 * \param[in] ptr  Source bytes.
 * \param[in] len  Number of bytes to write.
 * \return Bytes written on success, \c -1 on partial write.
 */
__attribute__((used)) int _write(const int file, const void *const ptr, const size_t len)
{
    (void)file;
#if defined(PLATFORM_HAS_DEBUG)
    size_t bytes_written = 0;

    if (debug_bmp) {
        bytes_written = debug_serial_debug_write(ptr, len);
    }
#if defined(ENABLE_SEGGER_RTT)
    else {
        bytes_written = SEGGER_RTT_Write(0, ptr, len);
    }
#endif

    return (bytes_written == len) ? (int)bytes_written : -1;
#else
    (void)ptr;
    return len;
#endif
}
#endif
