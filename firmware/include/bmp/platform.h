/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* This file provides the platform specific declarations for the native implementation. */

#ifndef PLATFORMS_MIOLINK_PLATFORM_H
#define PLATFORMS_MIOLINK_PLATFORM_H

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "pico/unique_id.h"
#include "timing_rp2040.h"
#include "hardware/pio.h"

#define TARGET_SWD_IDLE_CYCLES (8)
#if (TARGET_SWD_IDLE_CYCLES < 8)
#error "TARGET_SWD_IDLE_CYCLES should be at least 8"
#endif

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp;
#define PLATFORM_IDENT              "(Unknown MioLink, ENABLE_DEBUG=1) "
#define PLATFORM_IDENT_MIOLINK      "MioLink, ENABLE_DEBUG=1"
#define PLATFORM_IDENT_MIOLINK_PICO "MioLink_Pico, ENABLE_DEBUG=1"
#define PLATFORM_IDENT_PICO         "Pico, ENABLE_DEBUG=1"
#define PLATFORM_IDENT_PICO_W       "Pico W, ENABLE_DEBUG=1"
#else
#define PLATFORM_IDENT              "(Unknown MioLink) "
#define PLATFORM_IDENT_MIOLINK      "MioLink"
#define PLATFORM_IDENT_MIOLINK_PICO "MioLink_Pico"
#define PLATFORM_IDENT_PICO         "Pico"
#define PLATFORM_IDENT_PICO_W       "Pico W"
#endif

#define BOARD_IDENT_LENGTH (256U)

#define PLATFORM_MIOLINK
#define PLATFORM_MIOLINK_REV_A (1)
#define PLATFORM_MIOLINK_REV_B (2)

#define PLATFORM_HAS_TRACESWO
#define PLATFORM_HAS_CUSTOM_COMMANDS
#define PLATFORM_HAS_POWER_SWITCH

#define GDB_ENDPOINT_NOTIF (0x84)
#define GDB_ENDPOINT       (0x01)

#define SERIAL_ENDPOINT_NOTIF (0x85)
#define SERIAL_ENDPOINT       (0x02)

#ifdef PLATFORM_HAS_TRACESWO
#define SWO_ENCODING (2)
#define SWO_ENDPOINT (0x83)
#endif

#define PIN_NOT_CONNECTED (0xFF)

/* Common pins */
#define HWTYPE_PIN_0 (16) /* If 1 - MioLink or Pico/Pico W, if 0 - MioLink_Pico */

#define HWVERSION_PIN_0 (15)
#define HWVERSION_PIN_1 (14)

#define HWVERSION_PICO (0x03 + 1) /* All bits are 1 + 1 */

/* MioLink revA pins */
#define MIOLINK_REVA_TARGET_VOLTAGE_ADC_CHANNEL (3)
#define MIOLINK_REVA_TARGET_VOLTAGE_ENABLE_PIN  (3)
#define MIOLINK_REVA_TARGET_VOLTAGE_FAULT_PIN   (1)

#define MIOLINK_REVA_LED_ACT_PIN (13)
#define MIOLINK_REVA_LED_ERR_PIN (10)
#define MIOLINK_REVA_LED_SER_PIN (12)

#define MIOLINK_REVA_TARGET_TCK_PIN     (24)
#define MIOLINK_REVA_TARGET_TDO_PIN     (17)
#define MIOLINK_REVA_TARGET_TDI_PIN     (28)
#define MIOLINK_REVA_TARGET_TMS_PIN     (26)
#define MIOLINK_REVA_TARGET_TMS_DIR_PIN (27)

#define MIOLINK_REVA_TARGET_UART_TX_PIN (8)
#define MIOLINK_REVA_TARGET_UART_RX_PIN (21)

#define MIOLINK_REVA_TARGET_NRST_PIN (18)

/* MioLink revB pins */
#define MIOLINK_REVB_TARGET_VOLTAGE_ADC_CHANNEL (3)
#define MIOLINK_REVB_TARGET_VOLTAGE_ENABLE_PIN  (3)
#define MIOLINK_REVB_TARGET_VOLTAGE_FAULT_PIN   (1)

#define MIOLINK_REVB_LED_ACT_PIN (13)
#define MIOLINK_REVB_LED_ERR_PIN (10)
#define MIOLINK_REVB_LED_SER_PIN (12)

#define MIOLINK_REVB_TARGET_TCK_PIN     (24)
#define MIOLINK_REVB_TARGET_TDO_PIN     (17)
#define MIOLINK_REVB_TARGET_TDI_PIN     (28)
#define MIOLINK_REVB_TARGET_TMS_PIN     (26)
#define MIOLINK_REVB_TARGET_TMS_DIR_PIN (25)

#define MIOLINK_REVB_TARGET_UART_TX_PIN (8)
#define MIOLINK_REVB_TARGET_UART_RX_PIN (21)

#define MIOLINK_REVB_TARGET_NRST_PIN (18)

/* MioLink_pico pins */
#define MIOLINK_PICO_TARGET_VOLTAGE_ADC_CHANNEL (1)
#define MIOLINK_PICO_TARGET_VOLTAGE_ENABLE_PIN  (3)
#define MIOLINK_PICO_TARGET_VOLTAGE_FAULT_PIN   (1)

#define MIOLINK_PICO_LED_ACT_PIN (13)
#define MIOLINK_PICO_LED_ERR_PIN (10)
#define MIOLINK_PICO_LED_SER_PIN (12)

#define MIOLINK_PICO_TARGET_TCK_PIN     (20)
#define MIOLINK_PICO_TARGET_TDO_PIN     (17)
#define MIOLINK_PICO_TARGET_TDI_PIN     (28)
#define MIOLINK_PICO_TARGET_TMS_PIN     (26)
#define MIOLINK_PICO_TARGET_TMS_DIR_PIN (19)

#define MIOLINK_PICO_TARGET_UART_TX_PIN (8)
#define MIOLINK_PICO_TARGET_UART_RX_PIN (21)

#define MIOLINK_PICO_TARGET_NRST_PIN (18)

/* Raspberry Pi Pico and Pico W pins */
#define PICO_LED_ACT_PIN (25)

#define PICO_TARGET_TCK_PIN (10)
#define PICO_TARGET_TDO_PIN (13)
#define PICO_TARGET_TDI_PIN (12)
#define PICO_TARGET_TMS_PIN (11)

#define PICO_TARGET_UART_TX_PIN (8)
#define PICO_TARGET_UART_RX_PIN (9)

#define PICO_TARGET_NRST_PIN (7)

#define PICO_DETECT_ADC_CHANNEL   (3)
#define PICO_DETECT_ADC_THRESHOLD (0x100)

#define SET_RUN_STATE(state)   running_status = (state)
#define SET_IDLE_STATE(state)  platform_set_idle_state(state)
#define SET_ERROR_STATE(state) platform_set_error_state(state)

#define USB_SERIAL_UART_MAIN    (uart1)
#define USB_SERIAL_UART_TDI_TDO (uart0)
#define TRACESWO_UART           (uart0)

#define USB_SERIAL_UART_MAIN_IRQ    (UART_IRQ_NUM(USB_SERIAL_UART_MAIN))
#define USB_SERIAL_UART_TDI_TDO_IRQ (UART_IRQ_NUM(USB_SERIAL_UART_TDI_TDO))
#define TRACESWO_UART_IRQ           (UART_IRQ_NUM(TRACESWO_UART))

#define USB_SERIAL_TRACESWO_DMA_IRQ (DMA_IRQ_0)

#define PLATFORM_PRIORITY_LOW    (tskIDLE_PRIORITY + 1)
#define PLATFORM_PRIORITY_NORMAL (tskIDLE_PRIORITY + 2)
#define PLATFORM_PRIORITY_HIGH   (tskIDLE_PRIORITY + 3)

bool platform_target_is_power_ok(void);

typedef enum {
	PLATFORM_DEVICE_TYPE_NOT_SET = 0,
	PLATFORM_DEVICE_TYPE_MIOLINK,
	PLATFORM_DEVICE_TYPE_MIOLINK_PICO,
	PLATFORM_DEVICE_TYPE_PICO,
	PLATFORM_DEVICE_TYPE_PICO_W,
} platform_device_type_t;

typedef struct {
	uint8_t tck;
	uint8_t tms;
	uint8_t tms_dir;
	uint8_t tdi;
	uint8_t tdo;
	uint8_t uart_tx;
	uint8_t uart_rx;
	uint8_t reset;
	bool reset_inverted;
} platform_target_pins_t;

typedef struct {
	uint8_t act;
	uint8_t ser;
	uint8_t err;
} platform_led_pins_t;

typedef struct {
	uint8_t enable_pin;
	uint8_t fault_pin;
	uint8_t adc_channel;
} platform_vtref_info_t;

void platform_update_sys_freq(void);
platform_device_type_t platform_hwtype(void);
void platform_update_hwtype(void);
const platform_target_pins_t *platform_get_target_pins(void);
const platform_led_pins_t *platform_get_led_pins(void);
const platform_vtref_info_t *platform_get_vtref_info(void);
void platform_make_board_ident(void);

void platform_vtref_init(void);

void platform_set_idle_state(bool state);
void platform_toggle_idle_state(void);
void platform_set_error_state(bool state);
void platform_set_serial_state(bool state);

#endif /* PLATFORMS_MIOLINK_PLATFORM_H */