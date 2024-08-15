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

#define USB_TASK_CORE_AFFINITY     (0x01) /* Core 0 only */

//#define USB_VID   (0x1209)
//#define USB_PID   (0x2730)

#define USB_VID   (0x1d50)
#define USB_PID   (0x6018)

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT "" FIRMWARE_VERSION

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
    return _desc_str;
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