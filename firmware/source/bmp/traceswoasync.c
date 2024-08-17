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

#include "atomic.h"
#include "timers.h"
#include "task.h"

#define TRACESWO_UART_NUMBER            (0)
#define TRACESWO_UART_HW                (uart_get_hw(UART_INSTANCE(TRACESWO_UART_NUMBER)))
#define TRACESWO_UART_IRQ               UART0_IRQ
#define TRACESWO_TASK_CORE_AFFINITY     (0x01) /* Core 0 only */

#define TRACESWO_RX_INT_FIFO_LEVEL      (16)

#define UART_DMA_RX_TOTAL_BUFFERS_SIZE  (16 * 1024)
#define UART_DMA_RX_NUMBER_OF_BUFFERS   (32)
#define UART_DMA_RX_BUFFER_SIZE         (UART_DMA_RX_TOTAL_BUFFERS_SIZE / UART_DMA_RX_NUMBER_OF_BUFFERS)

#if (UART_DMA_RX_NUMBER_OF_BUFFERS < 4)
#error "UART_DMA_RX_NUMBER_OF_BUFFERS should be at least 4"
#endif

#define UART_DMA_RX_BAUDRATE_THRESHOLD   (38400)
#define UART_DMA_RX_MIN_TIMEOUT          (5)
#define UART_DMA_RX_MAX_TIMEOUT          (200)

#define UART_NOTIFY_WAIT_PERIOD           (portMAX_DELAY)

static uint8_t uart_rx_buf[UART_DMA_RX_NUMBER_OF_BUFFERS][UART_DMA_RX_BUFFER_SIZE] = { 0 };
static uint32_t uart_rx_int_buf_pos = 0;
static uint32_t uart_rx_dma_buffer_full_mask = 0;
static uint32_t uart_rx_dma_current_buffer = 0;
static bool uart_rx_use_dma = false;
static xTimerHandle uart_rx_dma_timeout_timer;
static int uart_dma_rx_channel = -1;
static bool uart_rx_ongoing = false;

/* SWO decoding */
static bool decoding = false;

TaskHandle_t traceswo_task;

#define TRACESWO_VENDOR_INTERFACE   (0)

static bool __not_in_flash_func(send_to_usb)(uint8_t *data, const size_t len)
{
    if (tud_vendor_n_write_available(TRACESWO_VENDOR_INTERFACE) < len)
    {
        return false;
    }
    else
    {
        return (tud_vendor_n_write(TRACESWO_VENDOR_INTERFACE, data, len) == len);
    }
}

static void __not_in_flash_func(uart_rx_dma_timeout_callback)(TimerHandle_t xTimer)
{
    (void)(xTimer);
    xTaskNotify(traceswo_task, USB_SERIAL_DATA_UART_RX_TIMEOUT, eSetBits);
}

static void __not_in_flash_func(uart_rx_dma_init)(const uint32_t baudrate)
{
    TRACESWO_UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS;
    TRACESWO_UART_HW->imsc = 0;
    TRACESWO_UART_HW->ifls = 0;

    dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
    dma_channel_abort(uart_dma_rx_channel);
    dma_channel_acknowledge_irq0(uart_dma_rx_channel);

    dma_channel_config rx_config = dma_channel_get_default_config(uart_dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, DREQ_UART1_RX);

    dma_channel_configure(uart_dma_rx_channel,
                          &rx_config,
                          uart_rx_buf,
                          &(TRACESWO_UART_HW->dr),
                          UART_DMA_RX_BUFFER_SIZE,
                          false);

    uart_rx_use_dma = true;

    /* Calculate timer period - time to fill 3 rx buffers */
    uint32_t timer_period = (UART_DMA_RX_BUFFER_SIZE * 3 * 1000); /* 1000 - because we need milliseconds */
    timer_period /= (baudrate / 10);  /* byte rate, 1 data byte = 10 bits (8 data, 1 start and 1 stop) */
    if (timer_period < UART_DMA_RX_MIN_TIMEOUT)
    {
        timer_period = UART_DMA_RX_MIN_TIMEOUT;
    }
    else if (timer_period > UART_DMA_RX_MAX_TIMEOUT)
    {
        timer_period = UART_DMA_RX_MAX_TIMEOUT;
    }

    xTimerChangePeriod(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(timer_period), portMAX_DELAY);

    TRACESWO_UART_HW->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
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

        TRACESWO_UART_HW->imsc = 0;
        TRACESWO_UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS | UART_UARTDMACR_RXDMAE_BITS;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xTimerResetFromISR(uart_rx_dma_timeout_timer, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken;
    }
}

static void __not_in_flash_func(uart_rx_dma_process_buffers)(void)
{
    const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
    uint32_t clear_mask = 0;

    xTimerReset(uart_rx_dma_timeout_timer, pdMS_TO_TICKS(0));

    for (uint32_t i = 0; i < UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
    {
        const uint32_t buffer_bit = (1u << i);
        if (buffer_state & buffer_bit)
        {
            if (send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i])) != false)
            {
                //tud_vendor_n_write_flush(TRACESWO_VENDOR_INTERFACE);
                clear_mask |= buffer_bit;
            }
        }
    }

    Atomic_AND_u32(&uart_rx_dma_buffer_full_mask, ~clear_mask);
}

static bool __not_in_flash_func(uart_rx_dma_finish_receiving)(void)
{
    if (uart_rx_ongoing == false)
    {
        panic("Invalid uart_rx_ongoing!");
    }
    else
    {
        TRACESWO_UART_HW->dmacr = UART_UARTDMACR_TXDMAE_BITS;

        dma_channel_acknowledge_irq0(uart_dma_rx_channel);
        dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);

        const uint32_t buffer_state = uart_rx_dma_buffer_full_mask;
        uint32_t clear_mask = 0;

        for (uint32_t i = 0; i < UART_DMA_RX_NUMBER_OF_BUFFERS; i++)
        {
            const uint32_t buffer_bit = (1u << i);
            if (buffer_state & buffer_bit)
            {
                while (send_to_usb(uart_rx_buf[i], sizeof(uart_rx_buf[i])) == false)
                {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                //tud_vendor_n_write_flush(TRACESWO_VENDOR_INTERFACE);
                clear_mask |= buffer_bit;
            }
        }

        const uint32_t remaining = dma_hw->ch[uart_dma_rx_channel].transfer_count;
        const uint32_t data_in_buffer = sizeof(uart_rx_buf[0]) - remaining;

        while (send_to_usb(uart_rx_buf[uart_rx_dma_current_buffer], data_in_buffer) == false)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        tud_vendor_n_write_flush(TRACESWO_VENDOR_INTERFACE);

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
    if (uart_rx_int_buf_pos >= 64)
    {
        if (decoding)
        {
            /* write decoded swo packets to the uart port */
            traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos);

            uart_rx_int_buf_pos = 0;
        }
        else if (send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos) != false)
        {
            uart_rx_int_buf_pos = 0;
        }
    }

    uart_rx_ongoing = true;

    TRACESWO_UART_HW->imsc |= UART_UARTIMSC_RXIM_BITS;
}

static void __not_in_flash_func(uart_rx_int_finish)(void)
{
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
        if (decoding)
        {
            /* write decoded swo packets to the uart port */
            traceswo_decode(uart_rx_buf, uart_rx_int_buf_pos);
            tud_cdc_n_write_flush(USB_SERIAL_TARGET);
        }
        else
        {
            send_to_usb((uint8_t *) (uart_rx_buf), uart_rx_int_buf_pos);
            tud_vendor_n_write_flush(TRACESWO_VENDOR_INTERFACE);
        }

        uart_rx_int_buf_pos = 0;
    }

    uart_rx_ongoing = false;
    TRACESWO_UART_HW->imsc |= UART_UARTIMSC_RTIM_BITS;
}

void traceswo_init(uint32_t baudrate, uint32_t swo_chan_bitmask)
{
    /* Re-initialize UART */
    dma_channel_set_irq0_enabled(uart_dma_rx_channel, false);
    dma_channel_abort(uart_dma_rx_channel);
    dma_channel_acknowledge_irq0(uart_dma_rx_channel);

    gpio_set_function(TARGET_TDO_PIN, GPIO_FUNC_UART);
    uart_init(UART_INSTANCE(TRACESWO_UART_NUMBER), baudrate);

    //if (baudrate >= UART_DMA_RX_BAUDRATE_THRESHOLD)
    {
        //uart_rx_dma_init(baudrate);
    }
    //else
    {
        uart_rx_int_init();
    }

    traceswo_setmask(swo_chan_bitmask);
    decoding = swo_chan_bitmask != 0;
}

void traceswo_deinit(void)
{

}

_Noreturn static void __not_in_flash_func(traceswo_thread)(void* params);

void traceswo_task_init(void)
{
    BaseType_t status = xTaskCreate(traceswo_thread,
                                    "Trace",
                                    128*4,
                                    NULL,
                                    PLATFORM_PRIORITY_HIGH,
                                    &traceswo_task);

#if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(traceswo_thread, TRACESWO_TASK_CORE_AFFINITY);
#endif
}

static void __not_in_flash_func(aux_serial_receive_isr)(void);
static void __not_in_flash_func(uart_dma_handler)(void);

_Noreturn static void __not_in_flash_func(traceswo_thread)(void* params)
{
	uint32_t notificationValue = 0;

    uart_rx_dma_timeout_timer = xTimerCreate("UART_RX",
                                         pdMS_TO_TICKS(UART_DMA_RX_MAX_TIMEOUT),
                                         pdFALSE,
                                         NULL,
                                         uart_rx_dma_timeout_callback);

	irq_set_exclusive_handler(TRACESWO_UART_IRQ, aux_serial_receive_isr);
	irq_set_enabled(TRACESWO_UART_IRQ, true);

    uart_dma_rx_channel = dma_claim_unused_channel(true);

    //irq_set_exclusive_handler(DMA_IRQ_0, uart_dma_handler);
    //irq_set_enabled(DMA_IRQ_0, true);

	uint32_t wait_time = UART_NOTIFY_WAIT_PERIOD;

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
    }
}

static void __not_in_flash_func(aux_serial_receive_isr)(void)
{
    traceISR_ENTER();

    uint32_t uart_int_status = TRACESWO_UART_HW->mis;
    uint32_t notify_bits = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (uart_rx_use_dma == false)
    {
        if (uart_int_status & UART_UARTMIS_RXMIS_BITS)
        {
            for (uint32_t i = 0; i < (TRACESWO_RX_INT_FIFO_LEVEL - 1); i++)
            {
                if (TRACESWO_UART_HW->fr & UART_UARTFR_RXFE_BITS)
                {
                    break;
                }

                ((uint8_t*)uart_rx_buf)[uart_rx_int_buf_pos] = (uint8_t) TRACESWO_UART_HW->dr;
                if (++uart_rx_int_buf_pos >= sizeof(uart_rx_buf))
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

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        return;
    }

    xTaskNotifyFromISR(traceswo_task, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void __not_in_flash_func(uart_dma_handler)(void)
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

        if (++uart_rx_dma_current_buffer >= UART_DMA_RX_NUMBER_OF_BUFFERS)
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