/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011 Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2022-2024 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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
#include "platform.h"
#include "usb_serial.h"
#ifdef PLATFORM_HAS_TRACESWO
#include "traceswo.h"
#endif
#ifdef ENABLE_RTT
#include "rtt.h"
#endif
#include "tusb.h"

#include "FreeRTOS.h"
#include "atomic.h"
#include "timers.h"
#include "task.h"

#if ENABLE_SYSVIEW_TRACE
#include "SEGGER_RTT.h"
#endif

#define USB_SERIAL_UART_RX_INT_FIFO_LEVEL          (16)

#define USB_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE  (16 * 1024)
#define USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS   (32)
#define USB_SERIAL_UART_DMA_RX_BUFFER_SIZE         (USB_SERIAL_UART_DMA_RX_TOTAL_BUFFERS_SIZE / \
                                                    USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS)

#if (USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define USB_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD     (38400)
#define USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT            (5)
#define USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT            (200)

#define USB_SERIAL_UART_DMA_TX_BUFFER_SIZE            (256)
#define USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD  (2)

#define USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD            (3000)
#define USB_SERIAL_USB_TX_WAIT_FOR_AVAIL              (1)
#define USB_SERIAL_USB_TX_WAIT_MAX                    (200)

#define USB_SERIAL_TASK_CORE_AFFINITY                 (0x02) /* Core 0 only */
#define USB_SERIAL_TASK_STACK_SIZE                    (512)

static uint8_t uart_rx_buf[USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS][USB_SERIAL_UART_DMA_RX_BUFFER_SIZE] = { 0 };
static uint32_t uart_rx_int_buf_pos = 0;
static uint32_t uart_rx_dma_buffer_full_mask = 0;
static uint32_t uart_rx_dma_current_buffer = 0;
static bool uart_rx_use_dma = false;
static xTimerHandle uart_rx_dma_timeout_timer;
static int uart_dma_rx_channel = -1;
static bool uart_rx_ongoing = false;

static int uart_dma_tx_channel = -1;
uint8_t uart_tx_dma_buf[USB_SERIAL_UART_DMA_TX_BUFFER_SIZE] = { 0 };
bool uart_tx_dma_finished = false;
static bool uart_tx_ongoing = false;

TaskHandle_t usb_uart_task;

/* Sets externally */
bool use_uart_on_tdi_tdo = false;

static uint32_t current_uart_number = USB_SERIAL_UART_MAIN_NUMBER;

#define UART_HW                (uart_get_hw(UART_INSTANCE(current_uart_number)))

static void __not_in_flash_func(uart_rx_isr)(void);
static void __not_in_flash_func(uart_dma_handler)(void);

void __not_in_flash_func(usb_serial_update_led)(void)
{
    if (tud_cdc_n_connected(USB_SERIAL_TARGET) == false)
    {
        gpio_clear(PICO_GPIO_PORT, LED_SER_PIN);
    }
    else if ((uart_rx_ongoing) || (uart_tx_ongoing))
    {
        gpio_set(PICO_GPIO_PORT, LED_SER_PIN);
    }
    else
    {
        gpio_clear(PICO_GPIO_PORT, LED_SER_PIN);
    }
}

bool __not_in_flash_func(usb_serial_send_to_usb)(uint8_t *data, const size_t len, const bool flush)
{
    if (tud_cdc_n_connected(USB_SERIAL_TARGET) == false)
    {
        return true;
    }
    else
    {
        bool result = false;
        bool need_flush = flush;
        if (need_flush == false)
        {
            for (size_t i = 0; i < len; i++)
            {
                if (data[i] == '\n')
                {
                    need_flush = true;
                    break;
                }
            }
        }

        if (tud_cdc_n_write_available(USB_SERIAL_TARGET) < len)
        {
            platform_timeout_s timeout;
            platform_timeout_set(&timeout, USB_SERIAL_USB_TX_WAIT_MAX);

            size_t written = 0;
            while ((written < len) && (!platform_timeout_is_expired(&timeout)))
            {
                const size_t available = tud_cdc_n_write_available(USB_SERIAL_TARGET);

                if (available > 0)
                {
                    const size_t data_to_write = MIN(len - written, available);
                    if (tud_cdc_n_write(USB_SERIAL_TARGET, data + written, data_to_write) == data_to_write)
                    {
                        written += data_to_write;
                    }
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(USB_SERIAL_USB_TX_WAIT_FOR_AVAIL));
                }
            }

            result = (written == len);
        }
        else
        {
            result = (tud_cdc_n_write(USB_SERIAL_TARGET, data, len) == len);
        }

        if (need_flush)
        {
            tud_cdc_n_write_flush(USB_SERIAL_TARGET);
        }
        return result;
    }
}

static void __not_in_flash_func(uart_rx_dma_init)(const uint32_t baudrate)
{
    UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS;
    UART_HW->imsc = 0;
    UART_HW->ifls = 0;

    dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
    dma_channel_abort(uart_dma_rx_channel);
    dma_channel_acknowledge_irq0(uart_dma_rx_channel);

    dma_channel_config rx_config = dma_channel_get_default_config(uart_dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, uart_get_dreq(UART_INSTANCE(current_uart_number), false));

    dma_channel_configure(uart_dma_rx_channel,
                          &rx_config,
                          uart_rx_buf,
                          &(UART_HW->dr),
                          USB_SERIAL_UART_DMA_RX_BUFFER_SIZE,
                          false);

    uart_rx_use_dma = true;

    /* Calculate timer period - time to fill 2 rx buffers */
    uint32_t timer_period = (USB_SERIAL_UART_DMA_RX_BUFFER_SIZE * 2 * 1000); /* 1000 - because we need milliseconds */
    timer_period /= (baudrate / 10);  /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
    if (timer_period < USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT)
    {
        timer_period = USB_SERIAL_UART_DMA_RX_MIN_TIMEOUT;
    }
    else if (timer_period > USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT)
    {
        timer_period = USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT;
    }

    xTimerChangePeriod(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

    UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
}

static BaseType_t __not_in_flash_func(uart_rx_dma_start_receiving)(void)
{
    if (uart_rx_ongoing != false)
    {
        panic("Invalid uart_rx_ongoing!");
    }
    else
    {
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);

        dma_channel_set_read_addr(uart_dma_rx_channel, &(UART_HW->dr), false);
        dma_channel_set_write_addr(uart_dma_rx_channel, uart_rx_buf[0], false);
        dma_channel_set_trans_count(uart_dma_rx_channel, sizeof(uart_rx_buf[0]), true);

        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

        uart_rx_dma_current_buffer = 0;
        uart_rx_dma_buffer_full_mask = 0;
        uart_rx_ongoing = true;

        UART_HW->imsc = 0;
        UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS | UART_UARTDMACR_RXDMAE_BITS;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xTimerResetFromISR(uart_rx_dma_timeout_timer, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken;
    }
}

static void __not_in_flash_func(uart_rx_dma_process_buffers)(void)
{
    const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
    uint32_t clear_mask = 0;

    xTimerReset(uart_rx_dma_timeout_timer, 0);

    for (uint32_t i = 0; i < USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
    {
        const uint32_t buffer_bit = (1u << i);
        if (buffer_state & buffer_bit)
        {
            if (usb_serial_send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i]), false) != false)
            {
                clear_mask |= buffer_bit;
            }
        }
    }

    Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~clear_mask);
}

static void __not_in_flash_func(uart_rx_dma_timeout_callback)(TimerHandle_t xTimer)
{
    (void)(xTimer);
    xTaskNotify(usb_uart_task, USB_SERIAL_DATA_UART_RX_TIMEOUT, eSetBits);
}

static bool __not_in_flash_func(uart_rx_dma_finish_receiving)(void)
{
    if (uart_rx_ongoing == false)
    {
        panic("Invalid uart_rx_ongoing!");
    }
    else
    {
        UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS;

        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);

        const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
        uint32_t clear_mask = 0;

        for (uint32_t i = 0; i < USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
        {
            const uint32_t buffer_bit = (1u << i);
            if (buffer_state & buffer_bit)
            {
                usb_serial_send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i]), false);
                clear_mask |= buffer_bit;
            }
        }

        const uint32_t remaining = dma_hw->ch[uart_dma_rx_channel].transfer_count;
        const uint32_t data_in_buffer = sizeof(uart_rx_buf[0]) - remaining;

        usb_serial_send_to_usb(uart_rx_buf[uart_rx_dma_current_buffer], data_in_buffer, true);

        dma_channel_abort(uart_dma_rx_channel);
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);

        uart_rx_dma_current_buffer = 0;
        uart_rx_dma_buffer_full_mask = 0;
        uart_rx_ongoing = false;

        xTimerStop(uart_rx_dma_timeout_timer, 0);

        UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
    }

    return false;
}

static void __not_in_flash_func(uart_rx_int_init)(void)
{
    uart_rx_use_dma = false;

    /* Set RX FIFO level to 1/2 */
    UART_HW->ifls = (2u << UART_UARTIFLS_RXIFLSEL_LSB) | (0 << UART_UARTIFLS_TXIFLSEL_LSB);
    UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
}

static void __not_in_flash_func(uart_rx_int_process)(void)
{
    if (uart_rx_int_buf_pos > 0)
    {
        if (usb_serial_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, false) != false)
        {
            uart_rx_int_buf_pos = 0;
        }

        uart_rx_ongoing = true;
    }

    UART_HW->imsc |= UART_UARTIMSC_RXIM_BITS;
}

static void __not_in_flash_func(uart_rx_int_finish)(void)
{
    if (uart_rx_int_buf_pos > 0)
    {
        usb_serial_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, false);
        uart_rx_int_buf_pos = 0;
    }

    while ((UART_HW->fr & UART_UARTFR_RXFE_BITS) == 0)
    {
        ((uint8_t*)uart_rx_buf)[uart_rx_int_buf_pos++] = (uint8_t) UART_HW->dr;
    }

    if (uart_rx_int_buf_pos > 0)
    {
        usb_serial_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, true);
        uart_rx_int_buf_pos = 0;
    }

    uart_rx_ongoing = false;
    UART_HW->imsc |= UART_UARTIMSC_RTIM_BITS;
}

static void __not_in_flash_func(uart_tx_dma_send)(void)
{
    if (tud_cdc_n_available(USB_SERIAL_TARGET))
    {
        const uint32_t read_count = tud_cdc_n_read(USB_SERIAL_TARGET, uart_tx_dma_buf, sizeof(uart_tx_dma_buf));
        if (read_count != 0)
        {
            dma_channel_acknowledge_irq0(uart_dma_tx_channel);
            dma_channel_set_irq0_enabled(uart_dma_tx_channel, true);

            dma_channel_set_read_addr(uart_dma_tx_channel, uart_tx_dma_buf, false);
            dma_channel_set_write_addr(uart_dma_tx_channel, &(UART_HW->dr), false);
            dma_channel_set_trans_count(uart_dma_tx_channel, read_count, true);

            uart_tx_ongoing = true;

            dma_channel_set_irq0_enabled(uart_dma_tx_channel, true);
        }
        else if (uart_tx_ongoing)
        {
            uart_tx_dma_finished = true;
        }
    }
    else if (uart_tx_ongoing)
    {
        uart_tx_dma_finished = true;
    }
}

static bool __not_in_flash_func(uart_tx_dma_check_uart_finished)(void)
{
    if (UART_HW->fr & UART_UARTFR_BUSY_BITS)
    {
        return false;
    }
    else
    {
        return true;
    }
}


static void __not_in_flash_func(uart_update_config)(cdc_line_coding_t *line_coding)
{
    uint32_t stop_bits = 2;
    switch (line_coding->stop_bits)
    {
        case 0:
        case 1:
            stop_bits = 1;
            break;
        case 2:
        default:
            break;
    }

    uart_parity_t parity = UART_PARITY_NONE;
    switch (line_coding->parity)
    {
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

    if (line_coding->data_bits <= 8)
    {
        data_bits = line_coding->data_bits;
    }

    portENTER_CRITICAL();
    xTimerStop(uart_rx_dma_timeout_timer, 0);

    dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
    dma_channel_abort(uart_dma_rx_channel);
    dma_channel_acknowledge_irq0(uart_dma_rx_channel);

    dma_channel_set_irq0_enabled(uart_dma_tx_channel, false);
    dma_channel_abort(uart_dma_tx_channel);
    dma_channel_acknowledge_irq0(uart_dma_tx_channel);

    if ((use_uart_on_tdi_tdo != false) && (current_uart_number == USB_SERIAL_UART_MAIN_NUMBER))
    {
        if (traceswo_uart_is_used(USB_SERIAL_UART_TDI_TDO_NUMBER))
        {
            return;
        }

        irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);
        irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, false);
        irq_remove_handler(USB_SERIAL_UART_MAIN_IRQ, current_handler);

        current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ);
        irq_set_enabled(USB_SERIAL_UART_TDI_TDO_IRQ, false);
        irq_remove_handler(USB_SERIAL_UART_TDI_TDO_IRQ, current_handler);

        uart_deinit(UART_INSTANCE(USB_SERIAL_UART_MAIN_NUMBER));
        current_uart_number = USB_SERIAL_UART_TDI_TDO_NUMBER;
    }
    else if ((use_uart_on_tdi_tdo == false) && (current_uart_number == USB_SERIAL_UART_TDI_TDO_NUMBER))
    {
        if (traceswo_uart_is_used(USB_SERIAL_UART_MAIN_NUMBER))
        {
            return;
        }

        irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);
        irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, false);
        irq_remove_handler(USB_SERIAL_UART_MAIN_IRQ, current_handler);

        current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_TDI_TDO_IRQ);
        irq_set_enabled(USB_SERIAL_UART_TDI_TDO_IRQ, false);
        irq_remove_handler(USB_SERIAL_UART_TDI_TDO_IRQ, current_handler);

        uart_deinit(UART_INSTANCE(USB_SERIAL_UART_TDI_TDO_NUMBER));
        current_uart_number = USB_SERIAL_UART_MAIN_NUMBER;
    }

    if (current_uart_number == USB_SERIAL_UART_MAIN_NUMBER)
    {
        USB_SERIAL_UART_MAIN_PIN_SETUP();

        irq_handler_t current_handler = irq_get_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ);
        if (current_handler == NULL)
        {
            irq_set_exclusive_handler(USB_SERIAL_UART_MAIN_IRQ, uart_rx_isr);
        }
        irq_set_enabled(USB_SERIAL_UART_MAIN_IRQ, true);
    }
    else
    {
        USB_SERIAL_UART_TDI_TDO_PIN_SETUP();

        irq_handler_t current_handler = irq_get_exclusive_handler(UART0_IRQ);
        if (current_handler == NULL)
        {
            irq_set_exclusive_handler(UART0_IRQ, uart_rx_isr);
        }
        irq_set_enabled(UART0_IRQ, true);
    }

    dma_channel_config tx_config = dma_channel_get_default_config(uart_dma_tx_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);

    channel_config_set_dreq(&tx_config, uart_get_dreq(UART_INSTANCE(current_uart_number), true));

    dma_channel_configure(uart_dma_tx_channel,
                          &tx_config,
                          &(UART_HW->dr),
                          uart_tx_dma_buf,
                          sizeof(uart_tx_dma_buf),
                          false);

    uart_init(UART_INSTANCE(current_uart_number), line_coding->bit_rate);
    uart_set_format(UART_INSTANCE(current_uart_number), data_bits, stop_bits, parity);

    uart_rx_int_buf_pos = 0;
    uart_rx_ongoing = false;
    uart_tx_ongoing = false;

    if (line_coding->bit_rate >= USB_SERIAL_UART_DMA_RX_BAUDRATE_THRESHOLD)
    {
        uart_rx_dma_init(line_coding->bit_rate);
    }
    else
    {
        uart_rx_int_init();
    }

    portEXIT_CRITICAL();
}

void usb_serial_uart_release(const uint32_t uart_number)
{
    if (uart_number == current_uart_number)
    {
        if (current_uart_number == USB_SERIAL_UART_TDI_TDO_NUMBER)
        {
            use_uart_on_tdi_tdo = false;
        }
        else
        {
            use_uart_on_tdi_tdo = true;
        }

        cdc_line_coding_t line_coding = { 0 };
        tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

        uart_update_config(&line_coding);
    }
}

_Noreturn static void __not_in_flash_func(target_serial_thread)(void* params);

void usb_serial_init(void)
{
    BaseType_t status = xTaskCreate(target_serial_thread,
                                    "target_uart",
                                    USB_SERIAL_TASK_STACK_SIZE,
                                    NULL,
                                    PLATFORM_PRIORITY_HIGH,
                                    &usb_uart_task);

#if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(usb_uart_task, USB_SERIAL_TASK_CORE_AFFINITY);
#endif
}

#ifdef ENABLE_RTT
extern void rtt_serial_receive_callback(void);
#endif

_Noreturn static void __not_in_flash_func(target_serial_thread)(void* params)
{
	uint32_t notificationValue = 0;

    uart_rx_dma_timeout_timer = xTimerCreate("UART_RX",
                                         pdMS_TO_TICKS(USB_SERIAL_UART_DMA_RX_MAX_TIMEOUT),
                                         pdFALSE,
                                         NULL,
                                         uart_rx_dma_timeout_callback);

    uart_dma_tx_channel = dma_claim_unused_channel(true);
    uart_dma_rx_channel = dma_claim_unused_channel(true);

    irq_set_exclusive_handler(USB_SERIAL_TRACESWO_DMA_IRQ, uart_dma_handler);
    irq_set_enabled(USB_SERIAL_TRACESWO_DMA_IRQ, true);

    irq_set_priority(USB_SERIAL_TRACESWO_DMA_IRQ, 0);

	uint32_t wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

	while (1)
    {
        if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(wait_time)) == pdPASS)
        {
            if (notificationValue & USB_SERIAL_LINE_CODING_UPDATE)
            {
                cdc_line_coding_t line_coding = { 0 };
                tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

                uart_update_config(&line_coding);
            }

			if (notificationValue & (USB_SERIAL_DATA_UART_RX_AVAILABLE | USB_SERIAL_DATA_UART_RX_FLUSH))
			{
                if (notificationValue & USB_SERIAL_DATA_UART_RX_AVAILABLE)
                {
                    if (uart_rx_use_dma == false)
                    {
                        uart_rx_int_process();
                    }
                    else
                    {
                        panic("Invalid notification!");
                    }
                }

				if (notificationValue & USB_SERIAL_DATA_UART_RX_FLUSH)
				{
                    if (uart_rx_use_dma == false)
                    {
                        uart_rx_int_finish();
                    }
                    else
                    {
                        uart_rx_dma_process_buffers();
                    }
				}
            }

			if (notificationValue & USB_SERIAL_DATA_UART_TX_COMPLETE)
			{
                uart_tx_dma_send();
			}

            if ((notificationValue & USB_SERIAL_DATA_UART_RX_TIMEOUT) && (uart_rx_ongoing != false))
            {
                uart_rx_dma_finish_receiving();
            }

            if ((notificationValue & USB_SERIAL_DATA_RX) && (uart_tx_ongoing == false))
            {
#ifdef ENABLE_RTT
                if (rtt_enabled)
                {
		            rtt_serial_receive_callback();
	            }
                else
                {
                    uart_tx_dma_send();
                }
#else
                uart_tx_dma_send();
#endif
            }
        }

        if (uart_tx_dma_finished)
        {
            if (uart_tx_dma_check_uart_finished())
            {
                wait_time = USB_SERIAL_TASK_NOTIFY_WAIT_PERIOD;

                uart_tx_ongoing = false;
                uart_tx_dma_finished = false;
            }
            else
            {
                wait_time = USB_SERIAL_UART_DMA_TX_CHECK_FINISHED_PERIOD;
            }
        }

        if (((use_uart_on_tdi_tdo != false) && (current_uart_number == USB_SERIAL_UART_MAIN_NUMBER)) ||
            ((use_uart_on_tdi_tdo == false) && (current_uart_number == USB_SERIAL_UART_TDI_TDO_NUMBER)))
        {
            cdc_line_coding_t line_coding = { 0 };
            tud_cdc_n_get_line_coding(USB_SERIAL_TARGET, &line_coding);

            uart_update_config(&line_coding);
        }

        usb_serial_update_led();
    }
}

static void __not_in_flash_func(uart_rx_isr)(void)
{
    traceISR_ENTER();

    uint32_t uart_int_status = UART_HW->mis;
    uint32_t notify_bits = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (uart_rx_use_dma == false)
    {
        if (uart_int_status & UART_UARTMIS_RXMIS_BITS)
        {
            for (uint32_t i = 0; i < (USB_SERIAL_UART_RX_INT_FIFO_LEVEL - 1); i++)
            {
                if (UART_HW->fr & UART_UARTFR_RXFE_BITS)
                {
                    break;
                }

                ((uint8_t*)uart_rx_buf)[uart_rx_int_buf_pos] = (uint8_t) UART_HW->dr;
                if (++uart_rx_int_buf_pos >= sizeof(uart_rx_buf))
                {
                    uart_rx_int_buf_pos = 0;
                }
            }

            UART_HW->icr |= UART_UARTICR_RXIC_BITS;
            UART_HW->imsc &= ~UART_UARTIMSC_RXIM_BITS;
            notify_bits |= USB_SERIAL_DATA_UART_RX_AVAILABLE;
        }

        if (uart_int_status & UART_UARTMIS_RTMIS_BITS)
        {
            UART_HW->icr |= UART_UARTICR_RTIC_BITS;
            UART_HW->imsc &= ~UART_UARTIMSC_RTIM_BITS;
            notify_bits |= USB_SERIAL_DATA_UART_RX_FLUSH;
        }
    }
    else
    {
        xHigherPriorityTaskWoken = uart_rx_dma_start_receiving();

        usb_serial_update_led();

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        return;
    }

    xTaskNotifyFromISR(usb_uart_task, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void __not_in_flash_func(uart_dma_handler)(void)
{
    traceISR_ENTER();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (uart_dma_tx_channel == -1)
	{
        panic("uart_dma_tx_channel is -1!");
	}
	else if (dma_channel_get_irq0_status(uart_dma_tx_channel))
	{
        dma_channel_set_irq0_enabled(uart_dma_tx_channel, false);
		dma_channel_acknowledge_irq0(uart_dma_tx_channel);

		xTaskNotifyFromISR(usb_uart_task, USB_SERIAL_DATA_UART_TX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

    if (uart_dma_rx_channel == -1)
    {
        panic("uart_dma_rx_channel is -1!");
    }
    else if (dma_channel_get_irq0_status(uart_dma_rx_channel))
    {
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);

        uart_rx_dma_buffer_full_mask |= (1u << uart_rx_dma_current_buffer);

        if (++uart_rx_dma_current_buffer >= USB_SERIAL_UART_DMA_RX_NUMBER_OF_BUFFERS)
        {
            uart_rx_dma_current_buffer = 0;
        }

        dma_channel_set_read_addr(uart_dma_rx_channel, &(UART_HW->dr), false);
        dma_channel_set_write_addr(uart_dma_rx_channel, uart_rx_buf[uart_rx_dma_current_buffer], false);
        dma_channel_set_trans_count(uart_dma_rx_channel, sizeof(uart_rx_buf[uart_rx_dma_current_buffer]), true);

        dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);
-
        xTaskNotifyFromISR(usb_uart_task, USB_SERIAL_DATA_UART_RX_FLUSH, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    traceswo_uart_dma_handler();
}

void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{
    usb_serial_send_to_usb((uint8_t *) data, len, true);
}

#if ENABLE_DEBUG == 1
size_t debug_serial_debug_write(const char *buf, const size_t len)
{
    return send_to_usb((uint8_t*)buf, len) ? len : 0;
}

__attribute__((used)) int _write(const int file, const void *const ptr, const size_t len)
{
    (void)file;
#ifdef PLATFORM_HAS_DEBUG
    if (debug_bmp)
		return debug_serial_debug_write(ptr, len);
    else
        SEGGER_RTT_Write(0, ptr, len);
#else
    (void)ptr;
#endif
    return len;
}
#endif