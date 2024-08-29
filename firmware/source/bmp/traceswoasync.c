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
#include "traceswo.h"
#include "tusb.h"

#include "hardware/uart.h"
#include "hardware/clocks.h"

#include "FreeRTOS.h"
#include "atomic.h"
#include "timers.h"
#include "task.h"

#define TRACESWO_UART_HW                     (uart_get_hw(UART_INSTANCE(TRACESWO_UART_NUMBER)))

#define TRACESWO_UART_RX_INT_FIFO_LEVEL      (16)

#define TRACESWO_UART_DMA_RX_TOTAL_BUFFERS_SIZE  (16 * 1024)
#define TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS   (32)
#define TRACESWO_UART_DMA_RX_BUFFER_SIZE         (TRACESWO_UART_DMA_RX_TOTAL_BUFFERS_SIZE / TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS)

#if (TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define TRACESWO_UART_DMA_RX_BAUDRATE_THRESHOLD   (38400)
#define TRACESWO_UART_DMA_RX_MIN_TIMEOUT          (5)
#define TRACESWO_UART_DMA_RX_MAX_TIMEOUT          (200)

#define TRACESWO_USB_TX_WAIT_FOR_AVAIL     (1)
#define TRACESWO_USB_TX_WAIT_MAX           (200)

#define TRACESWO_TASK_NOTIFY_WAIT_PERIOD   (portMAX_DELAY)

#define TRACESWO_VENDOR_INTERFACE          (0)

#define TRACESWO_DECODE_THRESHOLD          (64)

#define TRACESWO_TASK_CORE_AFFINITY          (0x02) /* Core 0 only */
#define TRACESWO_TASK_STACK_SIZE             (512)

static uint8_t uart_rx_buf[TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS][TRACESWO_UART_DMA_RX_BUFFER_SIZE] = {0 };
static uint32_t uart_rx_int_buf_pos = 0;
static uint32_t uart_rx_dma_buffer_full_mask = 0;
static uint32_t uart_rx_dma_current_buffer = 0;
static bool uart_rx_use_dma = false;
static xTimerHandle uart_rx_dma_timeout_timer;
static int uart_dma_rx_channel = -1;
static bool uart_rx_ongoing = false;

static bool traceswo_initialized = false;

/* SWO decoding */
static bool traceswo_decoding = false;

TaskHandle_t traceswo_task;

static void __not_in_flash_func(uart_rx_isr)(void);
static void __not_in_flash_func(uart_dma_handler)(void);

void __not_in_flash_func(traceswo_update_led)(void)
{
    if (uart_rx_ongoing)
    {
        gpio_set(PICO_GPIO_PORT, LED_SER_PIN);
    }
    else
    {
        gpio_clear(PICO_GPIO_PORT, LED_SER_PIN);
    }
}

static bool __not_in_flash_func(usb_trace_send_to_usb)(uint8_t *data, const size_t len, const bool flush)
{
    bool result = false;

    if (tud_vendor_n_write_available(TRACESWO_VENDOR_INTERFACE) < len)
    {
        platform_timeout_s timeout;
        platform_timeout_set(&timeout, TRACESWO_USB_TX_WAIT_MAX);

        size_t written = 0;
        while ((written < len) && (!platform_timeout_is_expired(&timeout)))
        {
            const size_t available = tud_vendor_n_write_available(TRACESWO_VENDOR_INTERFACE);

            if (available > 0)
            {
                const size_t data_to_write = MIN(len - written, available);
                if (tud_vendor_n_write(TRACESWO_VENDOR_INTERFACE, data + written, data_to_write) == data_to_write)
                {
                    written += data_to_write;
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(TRACESWO_USB_TX_WAIT_FOR_AVAIL));
            }
        }

        result = (written == len);
    }
    else
    {
        result = (tud_vendor_n_write(TRACESWO_VENDOR_INTERFACE, data, len) == len);
    }

    if (flush)
    {
        tud_vendor_n_write_flush(TRACESWO_VENDOR_INTERFACE);
    }

    return result;
}

static void __not_in_flash_func(uart_rx_dma_init)(const uint32_t baudrate)
{
    TRACESWO_UART_HW->dmacr = 0;
    TRACESWO_UART_HW->imsc = UART_UARTIMSC_OEIM_BITS;
    TRACESWO_UART_HW->ifls = 0;

    dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
    dma_channel_abort(uart_dma_rx_channel);
    dma_channel_acknowledge_irq0(uart_dma_rx_channel);

    dma_channel_config rx_config = dma_channel_get_default_config(uart_dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, uart_get_dreq(UART_INSTANCE(TRACESWO_UART_NUMBER), false));
    channel_config_set_high_priority(&rx_config, true);

    dma_channel_configure(uart_dma_rx_channel,
                          &rx_config,
                          uart_rx_buf,
                          &(TRACESWO_UART_HW->dr),
                          TRACESWO_UART_DMA_RX_BUFFER_SIZE,
                          false);

    uart_rx_use_dma = true;

    /* Calculate timer period - time to fill 2 rx buffers */
    uint32_t timer_period = (TRACESWO_UART_DMA_RX_BUFFER_SIZE * 2 * 1000); /* 1000 - because we need milliseconds */
    timer_period /= (baudrate / 10);  /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
    if (timer_period < TRACESWO_UART_DMA_RX_MIN_TIMEOUT)
    {
        timer_period = TRACESWO_UART_DMA_RX_MIN_TIMEOUT;
    }
    else if (timer_period > TRACESWO_UART_DMA_RX_MAX_TIMEOUT)
    {
        timer_period = TRACESWO_UART_DMA_RX_MAX_TIMEOUT;
    }

    xTimerChangePeriod(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

    TRACESWO_UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS | UART_UARTIMSC_OEIM_BITS;
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

        dma_channel_set_read_addr(uart_dma_rx_channel, &(TRACESWO_UART_HW->dr), false);
        dma_channel_set_write_addr(uart_dma_rx_channel, uart_rx_buf[0], false);
        dma_channel_set_trans_count(uart_dma_rx_channel, sizeof(uart_rx_buf[0]), true);

        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

        uart_rx_dma_current_buffer = 0;
        uart_rx_dma_buffer_full_mask = 0;
        uart_rx_ongoing = true;

        TRACESWO_UART_HW->imsc = UART_UARTIMSC_OEIM_BITS;
        TRACESWO_UART_HW->dmacr = UART_UARTDMACR_RXDMAE_BITS;

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

    for (uint32_t i = 0; i < TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
    {
        const uint32_t buffer_bit = (1u << i);
        if (buffer_state & buffer_bit)
        {
            if (traceswo_decoding)
            {
                traceswo_decode(uart_rx_buf[i], sizeof(uart_rx_buf[i]), true);
                clear_mask |= buffer_bit;
            }
            else if (usb_trace_send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i]), true) != false)
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
    xTaskNotify(traceswo_task, USB_SERIAL_DATA_UART_RX_TIMEOUT, eSetBits);
}

static bool __not_in_flash_func(uart_rx_dma_finish_receiving)(void)
{
    if (uart_rx_ongoing == false)
    {
        panic("Invalid uart_rx_ongoing!");
    }
    else
    {
        TRACESWO_UART_HW->dmacr = 0;

        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);

        const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;

        for (uint32_t i = 0; i < TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
        {
            const uint32_t buffer_bit = (1u << i);
            if (buffer_state & buffer_bit)
            {
                if (traceswo_decoding)
                {
                    traceswo_decode(uart_rx_buf[i], sizeof(uart_rx_buf[i]), false);
                }
                else
                {
                    usb_trace_send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i]), false);
                }
            }
        }

        const uint32_t remaining = dma_hw->ch[uart_dma_rx_channel].transfer_count;
        const uint32_t data_in_buffer = sizeof(uart_rx_buf[0]) - remaining;

        if (traceswo_decoding)
        {
           // traceswo_decode(uart_rx_buf[uart_rx_dma_current_buffer], data_in_buffer, true);
        }
        else
        {
            usb_trace_send_to_usb(uart_rx_buf[uart_rx_dma_current_buffer], data_in_buffer, true);
        }

        dma_channel_abort(uart_dma_rx_channel);
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);

        uart_rx_dma_current_buffer = 0;
        uart_rx_dma_buffer_full_mask = 0;
        uart_rx_ongoing = false;

        xTimerStop(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(0));

        TRACESWO_UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
    }

    return false;
}

static void __not_in_flash_func(uart_rx_int_init)(void)
{
    uart_rx_use_dma = false;

    /* Set RX FIFO level to 1/2 */
    TRACESWO_UART_HW->ifls = (2u << UART_UARTIFLS_RXIFLSEL_LSB) | (0 << UART_UARTIFLS_TXIFLSEL_LSB);
    TRACESWO_UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
}

static void __not_in_flash_func(uart_rx_int_process)(void)
{
    if (uart_rx_int_buf_pos > 0)
    {
        if ((traceswo_decoding) && (uart_rx_int_buf_pos >= TRACESWO_DECODE_THRESHOLD))
        {
            /* write decoded swo packets to the uart port */
            traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos, false);

            uart_rx_int_buf_pos = 0;
        }
        else if (usb_trace_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, false) != false)
        {
            uart_rx_int_buf_pos = 0;
        }

        uart_rx_ongoing = true;
    }

    TRACESWO_UART_HW->imsc |= UART_UARTIMSC_RXIM_BITS;
}

static void __not_in_flash_func(uart_rx_int_finish)(void)
{
    if (uart_rx_int_buf_pos > 0)
    {
        usb_trace_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, false);
        uart_rx_int_buf_pos = 0;
    }

    while ((TRACESWO_UART_HW->fr & UART_UARTFR_RXFE_BITS) == 0)
    {
        ((uint8_t *) uart_rx_buf)[uart_rx_int_buf_pos++] = (uint8_t) TRACESWO_UART_HW->dr;

        if (uart_rx_int_buf_pos >= sizeof(uart_rx_buf))
        {
            uart_rx_int_buf_pos = 0;
        }
    }

    if (uart_rx_int_buf_pos > 0)
    {
        if (traceswo_decoding)
        {
            /* write decoded swo packets to the uart port */
            traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos, true);
        }
        else
        {
            usb_trace_send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos, true);
        }

        uart_rx_int_buf_pos = 0;
    }

    uart_rx_ongoing = false;
    TRACESWO_UART_HW->imsc |= UART_UARTIMSC_RTIM_BITS;
}

void traceswo_init(uint32_t baudrate, uint32_t swo_chan_bitmask)
{
    usb_serial_uart_release(TRACESWO_UART_NUMBER);

    portENTER_CRITICAL();

    traceswo_deinit();

    /* Re-initialize UART */
    TRACESWO_UART_PIN_SETUP();

    irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_UART_IRQ);
    if (current_handler == NULL)
    {
        irq_set_exclusive_handler(TRACESWO_UART_IRQ, uart_rx_isr);
    }
    else
    {
        panic("TRACESWO UART IRQ handler was already set!");
    }
    irq_set_enabled(TRACESWO_UART_IRQ, true);
    irq_set_priority(TRACESWO_UART_IRQ, 0x80 - 1);

    uart_init(UART_INSTANCE(TRACESWO_UART_NUMBER), baudrate);

    if (baudrate >= TRACESWO_UART_DMA_RX_BAUDRATE_THRESHOLD)
    {
        uart_rx_dma_init(baudrate);
    }
    else
    {
        uart_rx_int_init();
    }

    traceswo_setmask(swo_chan_bitmask);
    traceswo_decoding = swo_chan_bitmask != 0;

    traceswo_initialized = true;

    portEXIT_CRITICAL();
}

void traceswo_deinit(void)
{
    if (traceswo_initialized)
    {
        portENTER_CRITICAL();

        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
        dma_channel_abort(uart_dma_rx_channel);
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);

        irq_handler_t current_handler = irq_get_exclusive_handler(TRACESWO_UART_IRQ);
        irq_set_enabled(TRACESWO_UART_IRQ, false);

        if (current_handler != NULL)
        {
            irq_remove_handler(TRACESWO_UART_IRQ, current_handler);
        }

        uart_deinit(UART_INSTANCE(TRACESWO_UART_NUMBER));

        uart_rx_int_buf_pos = 0;
        uart_rx_ongoing = false;

        traceswo_initialized = false;

        portEXIT_CRITICAL();
    }
}

bool traceswo_uart_is_used(const uint32_t uart_number)
{
    return (traceswo_initialized && (uart_number == TRACESWO_UART_NUMBER));
}

uint32_t traceswo_get_baudrate(void)
{
    const uint32_t ibrd = TRACESWO_UART_HW->ibrd;
    const uint32_t fbrd = TRACESWO_UART_HW->fbrd;
    return (4 * clock_get_hz(UART_CLOCK_NUM(UART_INSTANCE(TRACESWO_UART_NUMBER)))) / (64 * ibrd + fbrd);
}

_Noreturn static void __not_in_flash_func(traceswo_thread)(void* params);

void traceswo_task_init(void)
{
    BaseType_t status = xTaskCreate(traceswo_thread,
                                    "target_trace",
                                    TRACESWO_TASK_STACK_SIZE,
                                    NULL,
                                    PLATFORM_PRIORITY_HIGH,
                                    &traceswo_task);

#if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(traceswo_task, TRACESWO_TASK_CORE_AFFINITY);
#endif
}

_Noreturn static void __not_in_flash_func(traceswo_thread)(void* params)
{
	uint32_t notificationValue = 0;

    uart_rx_dma_timeout_timer = xTimerCreate("UART_RX",
                                         pdMS_TO_TICKS(TRACESWO_UART_DMA_RX_MAX_TIMEOUT),
                                         pdFALSE,
                                         NULL,
                                         uart_rx_dma_timeout_callback);


    uart_dma_rx_channel = dma_claim_unused_channel(true);

	uint32_t wait_time = TRACESWO_TASK_NOTIFY_WAIT_PERIOD;

	while (1)
    {
        if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(wait_time)) == pdPASS)
        {
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

            if ((notificationValue & USB_SERIAL_DATA_UART_RX_TIMEOUT) && (uart_rx_ongoing != false))
            {
                uart_rx_dma_finish_receiving();
            }
        }

        traceswo_update_led();
    }
}

static void __not_in_flash_func(uart_rx_isr)(void)
{
    traceISR_ENTER();

    uint32_t uart_int_status = TRACESWO_UART_HW->mis;
    uint32_t notify_bits = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uint32_t test = *(io_rw_32 *)(PPB_BASE + ARM_CPU_PREFIXED(NVIC_ISPR_OFFSET));

    if (uart_int_status & UART_UARTMIS_OEMIS_BITS)
    {
        panic("UART overrun!");
    }

    if (uart_rx_use_dma == false)
    {
        if (uart_int_status & UART_UARTMIS_RXMIS_BITS)
        {
            for (uint32_t i = 0; i < (TRACESWO_UART_RX_INT_FIFO_LEVEL - 1); i++)
            {
                if (TRACESWO_UART_HW->fr & UART_UARTFR_RXFE_BITS)
                {
                    break;
                }

                ((uint8_t*)uart_rx_buf)[uart_rx_int_buf_pos++] = (uint8_t) TRACESWO_UART_HW->dr;
                if (uart_rx_int_buf_pos >= sizeof(uart_rx_buf))
                {
                    uart_rx_int_buf_pos = 0;
                }
            }

            TRACESWO_UART_HW->icr |= UART_UARTICR_RXIC_BITS;
            TRACESWO_UART_HW->imsc &= ~UART_UARTIMSC_RXIM_BITS;
            notify_bits |= USB_SERIAL_DATA_UART_RX_AVAILABLE;
        }

        if (uart_int_status & UART_UARTMIS_RTMIS_BITS)
        {
            TRACESWO_UART_HW->icr |= UART_UARTICR_RTIC_BITS;
            TRACESWO_UART_HW->imsc &= ~UART_UARTIMSC_RTIM_BITS;
            notify_bits |= USB_SERIAL_DATA_UART_RX_FLUSH;
        }
    }
    else
    {
        xHigherPriorityTaskWoken = uart_rx_dma_start_receiving();

        traceswo_update_led();

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        return;
    }

    xTaskNotifyFromISR(traceswo_task, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void __not_in_flash_func(traceswo_uart_dma_handler)(void)
{
    traceISR_ENTER();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (uart_dma_rx_channel == -1)
    {
        panic("uart_dma_rx_channel is -1!");
    }
    else if (dma_channel_get_irq0_status(uart_dma_rx_channel))
    {
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
        dma_channel_acknowledge_irq0(uart_dma_rx_channel);

        uart_rx_dma_buffer_full_mask |= (1u << uart_rx_dma_current_buffer);

        if (++uart_rx_dma_current_buffer >= TRACESWO_UART_DMA_RX_NUMBER_OF_BUFFERS)
        {
            uart_rx_dma_current_buffer = 0;
        }

        dma_channel_set_read_addr(uart_dma_rx_channel, &(TRACESWO_UART_HW->dr), false);
        dma_channel_set_write_addr(uart_dma_rx_channel, uart_rx_buf[uart_rx_dma_current_buffer], false);
        dma_channel_set_trans_count(uart_dma_rx_channel, sizeof(uart_rx_buf[uart_rx_dma_current_buffer]), true);

        dma_channel_set_irq0_enabled(uart_dma_rx_channel, true);

        xTaskNotifyFromISR(traceswo_task, USB_SERIAL_DATA_UART_RX_FLUSH, eSetBits, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}