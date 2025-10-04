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

#include "general.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "pico/cyw43_arch.h"

#include "platform.h"

#include "version.h"

#define BOARD_IDENT_FORMAT "Black Magic Probe %s%s"

static platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_NOT_SET;

static const char *platform_ident_ptr = "";
char board_ident[BOARD_IDENT_LENGTH] = "";

/* MioLink revA pins */
static const platform_target_pins_t miolink_rev_a_target_pins = {.tck = MIOLINK_REVA_TARGET_TCK_PIN,
	.tms = MIOLINK_REVA_TARGET_TMS_PIN,
	.tms_dir = MIOLINK_REVA_TARGET_TMS_DIR_PIN,
	.tdi = MIOLINK_REVA_TARGET_TDI_PIN,
	.tdo = MIOLINK_REVA_TARGET_TDO_PIN,
	.uart_tx = MIOLINK_REVA_TARGET_UART_TX_PIN,
	.uart_rx = MIOLINK_REVA_TARGET_UART_RX_PIN,
	.reset = MIOLINK_REVA_TARGET_NRST_PIN,
	.reset_state = true};

static const platform_led_pins_t miolink_rev_a_led_pins = {
	.act = MIOLINK_REVA_LED_ACT_PIN, .ser = MIOLINK_REVA_LED_SER_PIN, .err = MIOLINK_REVA_LED_ERR_PIN};

static const platform_vtref_info_t miolink_rev_a_vtref_info = {.enable_pin = MIOLINK_REVA_TARGET_VOLTAGE_ENABLE_PIN,
	.fault_pin = MIOLINK_REVA_TARGET_VOLTAGE_FAULT_PIN,
	.adc_channel = MIOLINK_REVA_TARGET_VOLTAGE_ADC_CHANNEL};

/* MioLink revB pins */
static const platform_target_pins_t miolink_rev_b_target_pins = {.tck = MIOLINK_REVB_TARGET_TCK_PIN,
	.tms = MIOLINK_REVB_TARGET_TMS_PIN,
	.tms_dir = MIOLINK_REVB_TARGET_TMS_DIR_PIN,
	.tdi = MIOLINK_REVB_TARGET_TDI_PIN,
	.tdo = MIOLINK_REVB_TARGET_TDO_PIN,
	.uart_tx = MIOLINK_REVB_TARGET_UART_TX_PIN,
	.uart_rx = MIOLINK_REVB_TARGET_UART_RX_PIN,
	.reset = MIOLINK_REVB_TARGET_NRST_PIN,
	.reset_state = true};

static const platform_led_pins_t miolink_rev_b_led_pins = {
	.act = MIOLINK_REVB_LED_ACT_PIN, .ser = MIOLINK_REVB_LED_SER_PIN, .err = MIOLINK_REVB_LED_ERR_PIN};

static const platform_vtref_info_t miolink_rev_b_vtref_info = {.enable_pin = MIOLINK_REVB_TARGET_VOLTAGE_ENABLE_PIN,
	.fault_pin = MIOLINK_REVB_TARGET_VOLTAGE_FAULT_PIN,
	.adc_channel = MIOLINK_REVB_TARGET_VOLTAGE_ADC_CHANNEL};

/* MioLink_Pico pins */
static const platform_target_pins_t miolink_pico_target_pins = {.tck = MIOLINK_PICO_TARGET_TCK_PIN,
	.tms = MIOLINK_PICO_TARGET_TMS_PIN,
	.tms_dir = MIOLINK_PICO_TARGET_TMS_DIR_PIN,
	.tdi = MIOLINK_PICO_TARGET_TDI_PIN,
	.tdo = MIOLINK_PICO_TARGET_TDO_PIN,
	.uart_tx = MIOLINK_PICO_TARGET_UART_TX_PIN,
	.uart_rx = MIOLINK_PICO_TARGET_UART_RX_PIN,
	.reset = MIOLINK_PICO_TARGET_NRST_PIN,
	.reset_state = true};

static const platform_led_pins_t miolink_pico_led_pins = {
	.act = MIOLINK_PICO_LED_ACT_PIN, .ser = MIOLINK_PICO_LED_SER_PIN, .err = MIOLINK_PICO_LED_ERR_PIN};

static const platform_vtref_info_t miolink_pico_vtref_info = {.enable_pin = MIOLINK_PICO_TARGET_VOLTAGE_ENABLE_PIN,
	.fault_pin = MIOLINK_PICO_TARGET_VOLTAGE_FAULT_PIN,
	.adc_channel = MIOLINK_PICO_TARGET_VOLTAGE_ADC_CHANNEL};

/* Pico/Pico W pins */
static const platform_target_pins_t pico_target_pins = {.tck = PICO_TARGET_TCK_PIN,
	.tms = PICO_TARGET_TMS_PIN,
	.tms_dir = PIN_NOT_CONNECTED,
	.tdi = PICO_TARGET_TDI_PIN,
	.tdo = PICO_TARGET_TDO_PIN,
	.uart_tx = PICO_TARGET_UART_TX_PIN,
	.uart_rx = PICO_TARGET_UART_RX_PIN,
	.reset = PICO_TARGET_NRST_PIN,
	.reset_state = false};

static const platform_led_pins_t pico_led_pins = {
	.act = PICO_LED_ACT_PIN, .ser = PIN_NOT_CONNECTED, .err = PIN_NOT_CONNECTED};

static const platform_led_pins_t pico_w_led_pins = {
	.act = PIN_NOT_CONNECTED, .ser = PIN_NOT_CONNECTED, .err = PIN_NOT_CONNECTED};

platform_device_type_t platform_hwtype(void)
{
	return device_type;
}

void platform_update_hwtype(void)
{
	if (device_type == PLATFORM_DEVICE_TYPE_NOT_SET) {
		const int hwversion = platform_hwversion();

		if (hwversion == HWVERSION_PICO) {
			gpio_init(PICO_LED_ACT_PIN);
			gpio_set_dir(PICO_LED_ACT_PIN, GPIO_OUT);
			gpio_put(PICO_LED_ACT_PIN, false);

			adc_init();
			adc_gpio_init(26 + PICO_DETECT_ADC_CHANNEL);
			adc_select_input(PICO_DETECT_ADC_CHANNEL);

			/* Just drop the first measurement */
			adc_read();
			volatile uint16_t adc_result = adc_read();

			gpio_init(PICO_LED_ACT_PIN);
			gpio_init(26 + PICO_DETECT_ADC_CHANNEL);

			if (adc_result < PICO_DETECT_ADC_THRESHOLD) {
				cyw43_arch_init();
				device_type = PLATFORM_DEVICE_TYPE_PICO_W;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_PICO;
			}
		} else {
			gpio_init(HWTYPE_PIN_0);
			gpio_set_pulls(HWTYPE_PIN_0, true, false);

			for (uint32_t i = 0; i < 100000; i++)
				__nop();

			if (gpio_get(HWTYPE_PIN_0)) {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK_PICO;
			}
		}
	}
}

int platform_hwversion(void)
{
	static int hwversion = -1;

	if (hwversion == -1) {
		gpio_init(HWVERSION_PIN_0);
		gpio_init(HWVERSION_PIN_1);

		gpio_set_pulls(HWVERSION_PIN_0, true, false);
		gpio_set_pulls(HWVERSION_PIN_1, true, false);

		for (uint32_t i = 0; i < 100000; i++)
			__nop();

		hwversion = gpio_get(HWVERSION_PIN_1) ? (1 << 1) : 0;
		hwversion |= gpio_get(HWVERSION_PIN_0) ? (1 << 0) : 0;

		hwversion++;
	}

	return hwversion;
}

const platform_target_pins_t *platform_get_target_pins(void)
{
	const platform_target_pins_t *p_pins = NULL;

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK: {
		const int hw_version = platform_hwversion();
		if (hw_version == PLATFORM_MIOLINK_REV_A) {
			p_pins = &miolink_rev_a_target_pins;
		} else if (hw_version == PLATFORM_MIOLINK_REV_B) {
			p_pins = &miolink_rev_b_target_pins;
		} else {
			assert(false);
		}
		break;
	}

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_target_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_pins = &pico_target_pins;
		break;

	default:
		assert(false);
		break;
	}

	assert(p_pins != NULL);
	return p_pins;
}

const platform_led_pins_t *platform_get_led_pins(void)
{
	const platform_led_pins_t *p_pins = NULL;

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			p_pins = &miolink_rev_a_led_pins;
		} else {
			p_pins = &miolink_rev_b_led_pins;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_led_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
		p_pins = &pico_led_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_pins = &pico_w_led_pins;
		break;

	default:
		break;
	}

	return p_pins;
}

const platform_vtref_info_t *platform_get_vtref_info(void)
{
	const platform_vtref_info_t *p_vtref_info = NULL;

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			p_vtref_info = &miolink_rev_a_vtref_info;
		} else {
			p_vtref_info = &miolink_rev_b_vtref_info;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_vtref_info = &miolink_pico_vtref_info;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
	default:
		break;
	}

	return p_vtref_info;
}

void platform_make_board_ident(void)
{
	switch (platform_hwtype()) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_MIOLINK, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK;
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_MIOLINK_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK_PICO;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO;
		break;

	case PLATFORM_DEVICE_TYPE_PICO_W:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_PICO_W, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO_W;
		break;

	default:
		assert(false);
		break;
	}
}

const char *platform_ident(void)
{
	return platform_ident_ptr;
}