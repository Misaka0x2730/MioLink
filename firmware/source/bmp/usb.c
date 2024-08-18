/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
 * Modified 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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
#include "tusb.h"
#include "task.h"
#include "serialno.h"
#include "version.h"
#include "usb_serial.h"

#define USB_TASK_CORE_AFFINITY     (0x01) /* Core 0 only */

//#define USB_VID   (0x1209)
//#define USB_PID   (0x2730)

#define USB_VID   (0x1d50)
#define USB_PID   (0x6018)

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT "" FIRMWARE_VERSION

#define MAKE_CDC_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize) \
  /* Interface Associate */\
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Control Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE, _stridx,\
  /* CDC Header */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0120),\
  /* CDC Call */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0, (uint8_t)((_itfnum) + 1),\
  /* CDC ACM: support line request + send break */\
  4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 6,\
  /* CDC Union */\
  5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1),\
  /* Endpoint Notification */\
  7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size), 16,\
  /* CDC Data Interface */\
  9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum)+1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0,\
  /* Endpoint Out */\
  7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
  /* Endpoint In */\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define MAKE_DFU_DESCRIPTOR(_itfnum, _stridx) \
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 1, TUSB_CLASS_APPLICATION_SPECIFIC, 1, 1, _stridx,\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_APPLICATION_SPECIFIC, 1, 1, _stridx

#define DUMMY_DESC_SIZE (8+9)

#ifdef PLATFORM_HAS_TRACESWO
#define MAKE_TRACE_DESCRIPTOR(_itfnum, _stridx, _epin, _epsize) \
  8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0xFF, 0xFF, _stridx,\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0xFF, 0xFF, _stridx,\
  7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TRACE_DESC_SIZE (8+9+7)
#else
#define TRACE_DESC_SIZE (0)
#endif

enum
{
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,
    ITF_NUM_CDC,
    ITF_NUM_DFU = ITF_NUM_CDC,

#ifdef PLATFORM_HAS_TRACESWO
    ITF_NUM_TRACE,
#endif

    ITF_NUM_TOTAL
};

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
    .bcdDevice          = 0x0109,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

#define ENDPOINT_IN_BIT  (0x80)

#define ITF_CONFIG_LEN   (TUD_CONFIG_DESC_LEN + ((ITF_NUM_CDC / 2) * TUD_CDC_DESC_LEN) + DUMMY_DESC_SIZE + TRACE_DESC_SIZE)

static uint8_t const desc_fs_configuration[] =
{
    /* Config number, interface count, string index, total length, attribute, power in mA */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, ITF_CONFIG_LEN, 0x00, 500),

    /* 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
    MAKE_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, GDB_ENDPOINT_NOTIF, 8, GDB_ENDPOINT, GDB_ENDPOINT | ENDPOINT_IN_BIT, 64),

    /* 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
    MAKE_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 5, SERIAL_ENDPOINT_NOTIF, 8, SERIAL_ENDPOINT, SERIAL_ENDPOINT | ENDPOINT_IN_BIT, 64),

    MAKE_DFU_DESCRIPTOR(ITF_NUM_DFU, 6),

#ifdef PLATFORM_HAS_TRACESWO
    /* 3rd interface - TRACESWO */
    MAKE_TRACE_DESCRIPTOR(ITF_NUM_TRACE, 7, TRACE_ENDPOINT, 64),
#endif
};

static char const *string_desc_arr[] =
{
    (const char[]) { 0x09, 0x04, 0x00 }, // 0: is supported language is English (0x0409)
    "Black Magic Debug",
    BOARD_IDENT,
    serial_no,
    "Black Magic GDB Server",
    "Black Magic UART Port",
    "Black Magic DFU",
#ifdef PLATFORM_HAS_TRACESWO
    "Black Magic Trace Capture",
#endif
};

static uint16_t _desc_str[512] = { 0 };

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
    return (uint8_t const *)(&desc_device);
}

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_fs_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
    {
        return NULL;
    }
    else
    {
        size_t chr_count = 0;

        if (index == 0)
        {
            _desc_str[1] = string_desc_arr[0][0] + ((uint16_t)(string_desc_arr[0][1]) << 8);
            chr_count = 1;
        }
        else
        {
            const char *str = string_desc_arr[index];
            chr_count = strlen(str);
            const size_t max_count = (sizeof(_desc_str) / sizeof(_desc_str[0])) - 1;
            if (chr_count > max_count)
            {
                chr_count = max_count;
            }

            for (size_t i = 0; i < chr_count; i++)
            {
                _desc_str[1 + i] = str[i];
            }
        }

        // first byte is length (including header), second byte is string type
        _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    }
    return _desc_str;
}

extern TaskHandle_t usb_uart_task;
extern TaskHandle_t gdb_task;

void tud_cdc_rx_cb(uint8_t interface)
{
    if (interface == USB_SERIAL_GDB)
    {
        xTaskNotify(gdb_task, USB_SERIAL_DATA_RX, eSetBits);
    }
    else if (interface == USB_SERIAL_TARGET)
    {
        xTaskNotify(usb_uart_task, USB_SERIAL_DATA_RX, eSetBits);
    }
}

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts)
{
    (void) rts;
    (void) dtr;

    if (interface == USB_SERIAL_GDB)
    {
        xTaskNotify(gdb_task, USB_SERIAL_LINE_STATE_UPDATE, eSetBits);
    }
    else if (interface == USB_SERIAL_TARGET)
    {
        xTaskNotify(usb_uart_task, USB_SERIAL_LINE_STATE_UPDATE, eSetBits);
    }
}

void tud_cdc_line_coding_cb(uint8_t interface, cdc_line_coding_t const* p_line_coding)
{
    if (interface == USB_SERIAL_TARGET)
    {
        xTaskNotify(usb_uart_task, USB_SERIAL_LINE_CODING_UPDATE, eSetBits);
    }
}

_Noreturn static void usb_task_thread(void *param);

void blackmagic_usb_init(void)
{
    TaskHandle_t usb_task;
    BaseType_t status = xTaskCreate(usb_task_thread,
                                    "usb_task",
                                    configMINIMAL_STACK_SIZE*4,
                                    NULL,
                                    PLATFORM_PRIORITY_HIGH,
                                    &usb_task);

#if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(usb_task, USB_TASK_CORE_AFFINITY);
#endif
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