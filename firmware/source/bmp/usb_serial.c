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

/*
 * This file implements a the USB Communications Device Class - Abstract
 * Control Model (CDC-ACM) as defined in CDC PSTN subclass 1.2.
 * A Device Firmware Upgrade (DFU 1.1) class interface is provided for
 * field firmware upgrade.
 *
 * The device's unique id is used as the USB serial number string.
 *
 * Endpoint Usage
 *
 *     0 Control Endpoint
 * IN  1 GDB CDC DATA
 * OUT 1 GDB CDC DATA
 * IN  2 GDB CDC CTR
 * IN  3 UART CDC DATA
 * OUT 3 UART CDC DATA
 * OUT 4 UART CDC CTRL
 * In  5 Trace Capture
 *
 */

#include "general.h"
#include "platform.h"
#include "gdb_if.h"
#include "usb_serial.h"
#ifdef PLATFORM_HAS_TRACESWO
#include "traceswo.h"
#endif
#include "rtt.h"
#include "rtt_if.h"
#include "tusb.h"
#include "serialno.h"

static void aux_serial_receive_isr(void);
static void uart_dma_handler(void);

#include "tusb.h"
#include "task.h"

static void usb_task_thread(void *param);
static void gdb_thread(void* params);
static void target_serial_thread(void* params);

#define USB_SERIAL_DATA_RX                (0x01)
#define USB_SERIAL_LINE_STATE_UPDATE      (0x02)
#define USB_SERIAL_LINE_CODING_UPDATE     (0x04)
#define USB_SERIAL_DATA_UART_RX_AVAILABLE (0x08)
#define USB_SERIAL_DATA_UART_RX_FLUSH     (0x10)
#define USB_SERIAL_DATA_UART_TX_COMPLETE  (0x20)

typedef struct usb_serial_thread_info_s
{
	TaskHandle_t task;
	TaskFunction_t thread;
	char* name;
	configSTACK_DEPTH_TYPE stack_size;
	UBaseType_t priority;
	uint8_t interface_number;
    uint32_t notifyMask;
} usb_serial_thread_info_t;


static usb_serial_thread_info_t usb_serial_info[CFG_TUD_CDC] =
{
	{0, gdb_thread, "GDB", configMINIMAL_STACK_SIZE, PLATFORM_PRIORITY_NORMAL, 0, 0},
	{0, target_serial_thread, "Target", configMINIMAL_STACK_SIZE, PLATFORM_PRIORITY_NORMAL, 1, 0x3F}
};

static int dma_tx_channel = -1;

uint16_t usb_get_config(void)
{
    return tud_mounted() ? 1 : 0;
}

void tud_cdc_rx_cb(uint8_t interface)
{
    if ((interface < CFG_TUD_CDC) && (usb_serial_info[interface].task != NULL) && (usb_serial_info[interface].notifyMask & USB_SERIAL_DATA_RX))
    {
        xTaskNotify(usb_serial_info[interface].task, USB_SERIAL_DATA_RX, eSetBits);
    }
}

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts)
{
    (void) rts;
    (void) dtr;

    if ((interface < CFG_TUD_CDC) && (usb_serial_info[interface].task != NULL) && (usb_serial_info[interface].notifyMask & USB_SERIAL_LINE_STATE_UPDATE))
    {
        xTaskNotify(usb_serial_info[interface].task, USB_SERIAL_LINE_STATE_UPDATE, eSetBits);
    }
}

void tud_cdc_line_coding_cb(uint8_t interface, cdc_line_coding_t const* p_line_coding)
{
    if ((interface < CFG_TUD_CDC) && (usb_serial_info[interface].task != NULL) && (usb_serial_info[interface].notifyMask & USB_SERIAL_LINE_CODING_UPDATE))
    {
        xTaskNotify(usb_serial_info[interface].task, USB_SERIAL_LINE_CODING_UPDATE, eSetBits);
    }
}

bool gdb_serial_get_dtr(void)
{
    return tud_cdc_n_get_line_state(0) & 0x01;
}

void usb_serial_init(void)
{
	/*for (uint16_t i = 0; i < CFG_TUD_CDC; i++)
	{
		BaseType_t status = xTaskCreate(usb_serial_info[i].thread,
										usb_serial_info[i].name,
										usb_serial_info[i].stack_size*4,
										(void*)(&(usb_serial_info[i].interface_number)),
										usb_serial_info[i].priority,
										&(usb_serial_info[i].task));

        //vTaskCoreAffinitySet(usb_serial_info[i].task, 0x01);
	}*/
}

void usb_task_init(void)
{
    TaskHandle_t usb_task;
    BaseType_t status = xTaskCreate(usb_task_thread,
                         "usb_task",
                         configMINIMAL_STACK_SIZE*4,
                         NULL,
                         PLATFORM_PRIORITY_HIGH,
                         &usb_task);

    //vTaskCoreAffinitySet(usb_task, 0x02);
}

_Noreturn static void usb_task_thread(void *param)
{
    (void) param;

    tusb_init();

    while (1)
    {
        tud_task();
    }
}

_Noreturn static void gdb_thread(void* params)
{
	const uint8_t interface = *((uint8_t*)(params));
	uint32_t notificationValue = 0;
	bool isConnected = false;
    uint8_t rx_buf[128] = { 0 };

	while (1)
    {
        if (xTaskNotifyWait(pdFALSE, pdTRUE, &notificationValue, pdMS_TO_TICKS(portMAX_DELAY)) != pdFALSE)
        {
			/* Ignore line coding on GDB interface */
            if (notificationValue & USB_SERIAL_LINE_CODING_UPDATE)
            {
            }

			if (notificationValue & USB_SERIAL_LINE_STATE_UPDATE)
            {
            }

            if ((tud_cdc_n_connected(interface) == false) && (isConnected == true))
            {
                isConnected = false;
                continue;
            }

            if ((tud_cdc_n_connected(interface) == true) && (isConnected == false))
            {
                isConnected = true;
            }

            /*while (tud_cdc_n_available(interface))
            {
                const uint32_t read_count = tud_cdc_n_read(interface, rx_buf, sizeof(rx_buf));
                if (read_count == 0)
                {
                    break;
                }
            }*/
        }
    }
}

_Noreturn static void target_serial_thread(void* params)
{
	const uint8_t interface = *((uint8_t*)(params));
	uint32_t notificationValue = 0;
	bool isConnected = false;
	bool uart_tx_dma_finished = false;
	bool uart_tx_ongoing = false;

	UART_PIN_SETUP();

	uart_init(UART_INSTANCE, 38400);

	uart_set_irq_enables(UART_INSTANCE, true, false);

	irq_set_exclusive_handler(UART_IRQ, aux_serial_receive_isr);
	irq_set_enabled(UART_IRQ, true);

    dma_tx_channel = dma_claim_unused_channel(true);
    dma_channel_config tx_config = dma_channel_get_default_config(dma_tx_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, DREQ_UART1_TX);

    irq_set_exclusive_handler(DMA_IRQ_0, uart_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

	uint8_t rx_buf[128] = { 0 };
	uint32_t wait_time = portMAX_DELAY;

	while (1)
    {
        if (xTaskNotifyWait(0, UINT32_MAX, &notificationValue, pdMS_TO_TICKS(wait_time)) != pdFALSE)
        {
            if (notificationValue & USB_SERIAL_LINE_CODING_UPDATE)
            {
                cdc_line_coding_t line_coding = {0};
                tud_cdc_n_get_line_coding(interface, &line_coding);

                uint32_t stop_bits = 2;
                switch (line_coding.stop_bits)
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
                switch (line_coding.parity)
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

                if (line_coding.data_bits <= 8)
                {
                    data_bits = line_coding.data_bits;
                }

                uart_set_baudrate(UART_INSTANCE, line_coding.bit_rate);
                uart_set_format(UART_INSTANCE, data_bits, stop_bits, parity);
            }

            if ((tud_cdc_n_connected(interface) == false) && (isConnected == true))
            {
                isConnected = false;
                continue;
            }

            if ((tud_cdc_n_connected(interface) == true) && (isConnected == false))
            {
                isConnected = true;
            }

			if (notificationValue & (USB_SERIAL_DATA_UART_RX_AVAILABLE | USB_SERIAL_DATA_UART_RX_FLUSH))
			{
				while (uart_is_readable(UART_INSTANCE) && (tud_cdc_n_write_available(interface) > 0))
				{
					tud_cdc_n_write_char(interface, (char)(UART_HW->dr));
				}

				if (notificationValue & USB_SERIAL_DATA_UART_RX_FLUSH)
				{
					tud_cdc_n_write_flush(interface);
				}
			}

			if (notificationValue & USB_SERIAL_DATA_UART_TX_COMPLETE)
			{
				dma_channel_cleanup(dma_tx_channel);

				uart_tx_dma_finished = true;
			}

			if (uart_tx_dma_finished)
			{
				if (UART_HW->fr & UART_UARTFR_BUSY_BITS)
				{
					wait_time = 2;
					continue;
				}
				else
				{
					wait_time = portMAX_DELAY;
					uart_tx_ongoing = false;
					uart_tx_dma_finished = false;
				}
			}

			if (uart_tx_ongoing == false)
			{
				while (tud_cdc_n_available(interface))
				{
					const uint32_t read_count = tud_cdc_n_read(interface, rx_buf, sizeof(rx_buf));
					if (read_count == 0)
					{
						break;
					}

                    dma_channel_acknowledge_irq0(dma_tx_channel);
                    dma_channel_set_irq0_enabled(dma_tx_channel, true);

                    dma_channel_configure(dma_tx_channel, &tx_config,
                                          &(UART_HW->dr),
                                          rx_buf,
                                          read_count,  // transfer count
                                          true              // start immediately
                    );

					uart_tx_ongoing = true;
				}
			}
        }
    }
}

static void uart_dma_handler(void)
{
    traceISR_ENTER();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (dma_tx_channel == -1)
	{
		return;
	}
	else if (dma_channel_get_irq0_status(dma_tx_channel))
	{
        dma_channel_set_irq0_enabled(dma_tx_channel, false);
		dma_channel_acknowledge_irq0(dma_tx_channel);

		xTaskNotifyFromISR(usb_serial_info[USB_SERIAL_TARGET].task, USB_SERIAL_DATA_UART_TX_COMPLETE, eSetBits, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

static void aux_serial_receive_isr(void)
{
    traceISR_ENTER();

	uint32_t uart_int_status = UART_HW->mis;
	uint32_t notify_bits = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (uart_int_status & UART_UARTMIS_RXMIS_BITS)
	{
		UART_HW->icr |= UART_UARTICR_RXIC_BITS;
		notify_bits |= USB_SERIAL_DATA_UART_RX_AVAILABLE;
	}

	if (uart_int_status & UART_UARTMIS_RTMIS_BITS)
	{
		UART_HW->icr |= UART_UARTICR_RTIC_BITS;
		notify_bits |= USB_SERIAL_DATA_UART_RX_FLUSH;
	}

	xTaskNotifyFromISR(usb_serial_info[USB_SERIAL_TARGET].task, notify_bits, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{
}

//#define USB_VID   (0x1209)
//#define USB_PID   (0x2730)

#define USB_VID   (0x1d50)
#define USB_PID   (0x6018)

tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *)(&desc_device);
}

#define EPNUM_CDC_0_NOTIF   0x81
#define EPNUM_CDC_0_OUT     0x02
#define EPNUM_CDC_0_IN      0x82

#define EPNUM_CDC_1_NOTIF   0x83
#define EPNUM_CDC_1_OUT     0x04
#define EPNUM_CDC_1_IN      0x84

#ifdef PLATFORM_HAS_TRACESWO
#define EPNUM_CDC_2_NOTIF   0x85
#define EPNUM_CDC_2_OUT     0x06
#define EPNUM_CDC_2_IN      0x86
#endif

enum
{
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,
#ifdef PLATFORM_HAS_TRACESWO
    ITF_NUM_CDC_2,
	ITF_NUM_CDC_2_DATA,
#endif
    ITF_NUM_TOTAL
};

#define ITF_CONFIG_LEN   (TUD_CONFIG_DESC_LEN + ((ITF_NUM_TOTAL / 2) * TUD_CDC_DESC_LEN))

static uint8_t const desc_fs_configuration[] =
{
    /* Config number, interface count, string index, total length, attribute, power in mA */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, ITF_CONFIG_LEN, 0x00, 500),

    /* 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

    /* 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 5, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),

#ifdef PLATFORM_HAS_TRACESWO
    /* 2rd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_2, 6, EPNUM_CDC_2_NOTIF, 8, EPNUM_CDC_2_OUT, EPNUM_CDC_2_IN, 64),
#endif
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_fs_configuration;
}

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT "" FIRMWARE_VERSION

static char const *string_desc_arr[] =
        {
        (const char[]) { 0x09, 0x04, 0x00 }, // 0: is supported language is English (0x0409)
        "Black Magic Debug",
        BOARD_IDENT,
        serial_no,
        "Black Magic GDB Server",
        "Black Magic UART Port",
#ifdef PLATFORM_HAS_TRACESWO
        "Black Magic Trace Capture",
#endif
        };

static uint16_t _desc_str[64 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
    {
        return NULL;
    }
    else
    {
        const char *str = string_desc_arr[index];
        size_t chr_count = strlen(str);
        const size_t max_count = (sizeof(_desc_str) / sizeof(_desc_str[0])) - 1;
        if (chr_count > max_count)
        {
            chr_count = max_count;
        }

        for (size_t i = 0; i < chr_count; i++)
        {
            _desc_str[1 + i] = str[i];
        }

        // first byte is length (including header), second byte is string type
        _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    }

    /*switch ( index )
    {
        case STRID_LANGID:
            memcpy(&(_desc_str[1]), string_desc_arr[0], 2);
            chr_count = 1;
            break;

        case STRID_SERIAL:
        {
            chr_count = DFU_SERIAL_LENGTH - 1;
            const size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
            if (chr_count > max_count)
            {
                chr_count = max_count;
            }
            for (size_t i = 0; i < chr_count; i++)
            {
                _desc_str[1 + i] = serial_no[i];
            }
            break;
        }

        default:
        {
            // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
            // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

            if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            {
                return NULL;
            }

            const char *str = string_desc_arr[index];

            // Cap at max char
            chr_count = strlen(str);
            const size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
            if (chr_count > max_count)
            {
                chr_count = max_count;
            }

            // Convert ASCII string into UTF-16
            for (size_t i = 0; i < chr_count; i++)
            {
                _desc_str[1 + i] = str[i];
            }
            break;
        }
    }*/

    return _desc_str;
}