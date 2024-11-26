/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
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

#include "general.h"
#include "platform.h"
#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "serialno.h"
#include "version.h"
#include "usb_serial.h"

#define USB_TASK_CORE_AFFINITY (0x01) /* Core 0 only */
#define USB_TASK_STACK_SIZE    (512)

#define USB_VID (0x1d50)
#define USB_PID (0x6018)

#define MAKE_CDC_DESCRIPTOR(_itfnum, _stridx, _ep_notif, _ep_notif_size, _epout, _epin, _epsize)                       \
	/* Interface Associate */                                                                                          \
	8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 2, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,          \
		CDC_COMM_PROTOCOL_NONE, _stridx, /* CDC Control Interface */                                                   \
		9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL,               \
		CDC_COMM_PROTOCOL_NONE, _stridx,                                        /* CDC Header */                       \
		5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_HEADER, U16_TO_U8S_LE(0x0110), /* CDC Call */                         \
		5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_CALL_MANAGEMENT, 0,                                                   \
		(uint8_t)((_itfnum) + 1), /* CDC ACM: support line request + send break */                                     \
		4, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_ABSTRACT_CONTROL_MANAGEMENT, 2,           /* CDC Union */             \
		5, TUSB_DESC_CS_INTERFACE, CDC_FUNC_DESC_UNION, _itfnum, (uint8_t)((_itfnum) + 1), /* Endpoint Notification */ \
		7, TUSB_DESC_ENDPOINT, _ep_notif, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_ep_notif_size),                          \
		255,                                                                                  /* CDC Data Interface */ \
		9, TUSB_DESC_INTERFACE, (uint8_t)((_itfnum) + 1), 0, 2, TUSB_CLASS_CDC_DATA, 0, 0, 0, /* Endpoint Out */       \
		7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,             /* Endpoint In */        \
		7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define MAKE_DFU_DESCRIPTOR(_itfnum, _stridx)                                                          \
	8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 1, TUSB_CLASS_APPLICATION_SPECIFIC, 1, 1, _stridx, 9, \
		TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_APPLICATION_SPECIFIC, 1, 1, _stridx

#define DUMMY_DESC_SIZE (8 + 9)

#ifdef PLATFORM_HAS_TRACESWO
#define MAKE_TRACE_DESCRIPTOR(_itfnum, _stridx, _epin, _epsize)                                                     \
	8, TUSB_DESC_INTERFACE_ASSOCIATION, _itfnum, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0xFF, 0xFF, _stridx, 9,             \
		TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0xFF, 0xFF, _stridx, 7, TUSB_DESC_ENDPOINT, \
		_epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#define TRACE_DESC_SIZE (8 + 9 + 7)
#else
#define TRACE_DESC_SIZE (0)
#endif

enum {
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

tusb_desc_device_t const desc_device = {.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0210,

	// Use Interface Association Descriptor (IAD) for CDC
	// As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0109,

	.iManufacturer = 0x01,
	.iProduct = 0x02,
	.iSerialNumber = 0x03,

	.bNumConfigurations = 0x01};

#define ENDPOINT_IN_BIT (0x80)

#define ITF_CONFIG_LEN \
	(TUD_CONFIG_DESC_LEN + ((ITF_NUM_CDC / 2) * TUD_CDC_DESC_LEN) + DUMMY_DESC_SIZE + TRACE_DESC_SIZE)

static uint8_t const desc_fs_configuration[] = {
	/* Config number, interface count, string index, total length, attribute, power in mA */
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, ITF_CONFIG_LEN, 0x00, 500),

	/* 1st CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
	MAKE_CDC_DESCRIPTOR(
		ITF_NUM_CDC_0, 4, GDB_ENDPOINT_NOTIF | ENDPOINT_IN_BIT, 16, GDB_ENDPOINT, GDB_ENDPOINT | ENDPOINT_IN_BIT, 64),

	/* 2nd CDC: Interface number, string index, EP notification address and size, EP data address (out, in) and size. */
	MAKE_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 5, SERIAL_ENDPOINT_NOTIF | ENDPOINT_IN_BIT, 16, SERIAL_ENDPOINT,
		SERIAL_ENDPOINT | ENDPOINT_IN_BIT, 64),

	MAKE_DFU_DESCRIPTOR(ITF_NUM_DFU, 6),

#ifdef PLATFORM_HAS_TRACESWO
	/* 3rd interface - TRACESWO */
	MAKE_TRACE_DESCRIPTOR(ITF_NUM_TRACE, 7, SWO_ENDPOINT | ENDPOINT_IN_BIT, 64),
#endif
};

extern char board_ident[BOARD_IDENT_LENGTH];

static char const *string_desc_arr[] = {
	(const char[]){0x09, 0x04, 0x00}, // 0: is supported language is English (0x0409)
	"Black Magic Debug",
	board_ident,
	serial_no,
	"Black Magic GDB Server",
	"Black Magic UART Port",
	"Black Magic DFU",
#ifdef PLATFORM_HAS_TRACESWO
	"Black Magic Trace Capture",
#endif
};

static uint16_t string_descriptor[512] = {0};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void)
{
	return (uint8_t const *)(&desc_device);
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;
	return desc_fs_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void)langid;

	if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
		return NULL;
	}

	size_t chr_count = 0;

	if (index == 0) {
		string_descriptor[1] = string_desc_arr[0][0] + ((uint16_t)(string_desc_arr[0][1]) << 8);
		chr_count = 1;
	} else {
		const char *str = string_desc_arr[index];
		chr_count = strlen(str);
		const size_t max_count = (sizeof(string_descriptor) / sizeof(string_descriptor[0])) - 1;
		if (chr_count > max_count) {
			chr_count = max_count;
		}

		for (size_t i = 0; i < chr_count; i++) {
			string_descriptor[1 + i] = str[i];
		}
	}

	// first byte is length (including header), second byte is string type
	string_descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

	return string_descriptor;
}

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

#define MS_OS_20_DESC_LEN (10 + 8 + (8 + 20 + 128) * 2)

static const uint8_t desc_ms_os_20[] = {
	// Set header: length, type, Windows version, total length
	U16_TO_U8S_LE(0x000A),
	U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
	U32_TO_U8S_LE(0x06030000),
	U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

	// Config Subset header: length, type, configuration value, reserved, total length
	U16_TO_U8S_LE(0x0008),
	U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
	0x00,
	0x00,
	U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 10),

	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008),
	U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
	4,
	0,
	U16_TO_U8S_LE(0x009C),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(0x0014),
	U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
	'W',
	'I',
	'N',
	'U',
	'S',
	'B',
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, // sub-compatible

	// MS OS 2.0 Registry property descriptor: length, type
	U16_TO_U8S_LE(0x0080),
	U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
	U16_TO_U8S_LE(0x0001),
	U16_TO_U8S_LE(0x0028), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUID" in UTF-16
	'D',
	0x00,
	'e',
	0x00,
	'v',
	0x00,
	'i',
	0x00,
	'c',
	0x00,
	'e',
	0x00,
	'I',
	0x00,
	'n',
	0x00,
	't',
	0x00,
	'e',
	0x00,
	'r',
	0x00,
	'f',
	0x00,
	'a',
	0x00,
	'c',
	0x00,
	'e',
	0x00,
	'G',
	0x00,
	'U',
	0x00,
	'I',
	0x00,
	'D',
	0x00,
	0x00,
	0x00,
	U16_TO_U8S_LE(0x004E), // wPropertyDataLength
	// Vendor-defined Property Data:{76be5ca1-e304-4b32-be5f-d9369d3d201a}
	'{',
	0,
	'7',
	0,
	'6',
	0,
	'b',
	0,
	'e',
	0,
	'5',
	0,
	'c',
	0,
	'a',
	0,
	'1',
	0,
	'-',
	0,
	'e',
	0,
	'3',
	0,
	'0',
	0,
	'4',
	0,
	'-',
	0,
	'4',
	0,
	'b',
	0,
	'3',
	0,
	'2',
	0,
	'-',
	0,
	'b',
	0,
	'e',
	0,
	'5',
	0,
	'f',
	0,
	'-',
	0,
	'd',
	0,
	'9',
	0,
	'3',
	0,
	'6',
	0,
	'9',
	0,
	'd',
	0,
	'3',
	0,
	'd',
	0,
	'2',
	0,
	'0',
	0,
	'1',
	0,
	'a',
	0,
	'}',
	0,
	0,
	0,

	// Function Subset header: length, type, first interface, reserved, subset length
	U16_TO_U8S_LE(0x0008),
	U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
	5,
	0,
	U16_TO_U8S_LE(0x009C),

	// MS OS 2.0 Compatible ID descriptor: length, type, compatible ID, sub compatible ID
	U16_TO_U8S_LE(0x0014),
	U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
	'W',
	'I',
	'N',
	'U',
	'S',
	'B',
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, // sub-compatible

	// MS OS 2.0 Registry property descriptor: length, type
	U16_TO_U8S_LE(0x0080),
	U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
	U16_TO_U8S_LE(0x0001),
	U16_TO_U8S_LE(0x0028), // wPropertyDataType, wPropertyNameLength and PropertyName "DeviceInterfaceGUID" in UTF-16
	'D',
	0x00,
	'e',
	0x00,
	'v',
	0x00,
	'i',
	0x00,
	'c',
	0x00,
	'e',
	0x00,
	'I',
	0x00,
	'n',
	0x00,
	't',
	0x00,
	'e',
	0x00,
	'r',
	0x00,
	'f',
	0x00,
	'a',
	0x00,
	'c',
	0x00,
	'e',
	0x00,
	'G',
	0x00,
	'U',
	0x00,
	'I',
	0x00,
	'D',
	0x00,
	0x00,
	0x00,
	U16_TO_U8S_LE(0x004E), // wPropertyDataLength
	// Vendor-defined Property Data: {76be5ca1-e305-4b32-be5f-d9369d3d201a}
	'{',
	0,
	'7',
	0,
	'6',
	0,
	'b',
	0,
	'e',
	0,
	'5',
	0,
	'c',
	0,
	'a',
	0,
	'1',
	0,
	'-',
	0,
	'e',
	0,
	'3',
	0,
	'0',
	0,
	'5',
	0,
	'-',
	0,
	'4',
	0,
	'b',
	0,
	'3',
	0,
	'2',
	0,
	'-',
	0,
	'b',
	0,
	'e',
	0,
	'5',
	0,
	'f',
	0,
	'-',
	0,
	'd',
	0,
	'9',
	0,
	'3',
	0,
	'6',
	0,
	'9',
	0,
	'd',
	0,
	'3',
	0,
	'd',
	0,
	'2',
	0,
	'0',
	0,
	'1',
	0,
	'a',
	0,
	'}',
	0,
	0,
	0,
};

uint8_t const desc_bos[] = {
	// total length, number of device caps
	TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),

	// Microsoft OS 2.0 descriptor
	TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, 1)};

uint8_t const *tud_descriptor_bos_cb(void)
{
	return desc_bos;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
	// nothing to with DATA & ACK stage
	if (stage != CONTROL_STAGE_SETUP)
		return true;

	if (request->bRequest == 1 && request->wIndex == 7) {
		// Get Microsoft OS 2.0 compatible descriptor
		return tud_control_xfer(rhport, request, (void *)desc_ms_os_20, sizeof(desc_ms_os_20));
	}

	return false;
}

extern TaskHandle_t usb_uart_task;
extern TaskHandle_t gdb_task;

void tud_cdc_rx_cb(uint8_t interface)
{
	if (interface == USB_SERIAL_GDB) {
		xTaskNotify(gdb_task, USB_SERIAL_DATA_RX, eSetBits);
	} else if (interface == USB_SERIAL_TARGET) {
		xTaskNotify(usb_uart_task, USB_SERIAL_DATA_RX, eSetBits);
	}
}

void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool rts)
{
	(void)rts;
	(void)dtr;

	if (interface == USB_SERIAL_GDB) {
		xTaskNotify(gdb_task, USB_SERIAL_LINE_STATE_UPDATE, eSetBits);
	} else if (interface == USB_SERIAL_TARGET) {
		xTaskNotify(usb_uart_task, USB_SERIAL_LINE_STATE_UPDATE, eSetBits);
	}
}

void tud_cdc_line_coding_cb(uint8_t interface, cdc_line_coding_t const *p_line_coding)
{
	(void)p_line_coding;
	if (interface == USB_SERIAL_TARGET) {
		xTaskNotify(usb_uart_task, USB_SERIAL_LINE_CODING_UPDATE, eSetBits);
	}
}

static bool usb_config_updated = false;

void tud_mount_cb(void)
{
	usb_config_updated = true;
}

void tud_umount_cb(void)
{
	usb_config_updated = true;
}

uint16_t usb_get_config(void)
{
	return tud_mounted() ? 1 : 0;
}

bool usb_config_is_updated(void)
{
	return usb_config_updated;
}

void usb_config_clear_updated(void)
{
	usb_config_updated = false;
}

_Noreturn static void usb_task_thread(void *param);

void blackmagic_usb_init(void)
{
	TaskHandle_t usb_task;
#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(usb_task_thread, "usb_task", USB_TASK_STACK_SIZE, NULL,
		PLATFORM_PRIORITY_HIGH, USB_TASK_CORE_AFFINITY, &usb_task);
#else
	const BaseType_t result =
		xTaskCreate(usb_task_thread, "usb_task", USB_TASK_STACK_SIZE, NULL, PLATFORM_PRIORITY_HIGH, &usb_task);
#endif

	assert(result == pdPASS);
}

_Noreturn static void usb_task_thread(void *param)
{
	(void)param;

	tusb_init();

	while (1) {
		tud_task();
	}
}