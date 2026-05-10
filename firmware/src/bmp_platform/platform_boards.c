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
#include "hardware/sync.h"

#if defined(PICO_CYW43_SUPPORTED)
#include "hardware/adc.h"
#include "pico/cyw43_arch.h"
#endif

#include "platform.h"

#include "version.h"

#include "platform_ident_extra_info.h"

#define PLATFORM_IDENT_MIOLINK      "MioLink" PLATFORM_IDENT_EXTRA_INFO
#define PLATFORM_IDENT_MIOLINK_PICO "MioLink_Pico" PLATFORM_IDENT_EXTRA_INFO
#define PLATFORM_IDENT_PICO         "Pico" PLATFORM_IDENT_EXTRA_INFO
#define PLATFORM_IDENT_PICO_W       "Pico W" PLATFORM_IDENT_EXTRA_INFO

#if PLATFORM_IS_NON_MIOLINK_BOARD
#define BOARD_IDENT_NON_MIOLINK   PICO_BOARD PLATFORM_IDENT_EXTRA_INFO
#endif

#define BOARD_IDENT_FORMAT "Black Magic Probe (%s) %s"

#define PLATFORM_HWVERSION_UNKNOWN (-1)

/* Build-time board selection (PICO_BOARD): skip autodetect */
#if PLATFORM_AUTO_DETECT
static platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_NOT_SET;
#elif PLATFORM_BOARD_MIOLINK
static const platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_MIOLINK;
#elif PLATFORM_BOARD_MIOLINK_PICO
static const platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_MIOLINK_PICO;
#elif PLATFORM_PICO_BOARD
static const platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_PICO;
#elif PLATFORM_PICO_W_BOARD
static const platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_PICO_W;
#else
#error "Unable to determine device type"
#endif

static const char *platform_ident_ptr = "";
char board_ident[BOARD_IDENT_LENGTH] = "";

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK

/**
 * \brief GPIO maps for MioLink main board — hardware revision A.
 *
 * Numeric values come from board headers (\c MIOLINK_REVA_*).
 */

static const platform_target_pins_t miolink_rev_a_target_pins = {
	.tck = MIOLINK_REVA_TARGET_TCK_PIN,             /**< Target SWD/JTAG clock (TCK). */
	.tms = MIOLINK_REVA_TARGET_TMS_PIN,             /**< Target TMS / bidirectional SWDIO data. */
	.tms_dir = MIOLINK_REVA_TARGET_TMS_DIR_PIN,     /**< TMS/SWDIO direction control (when used). */
	.tdi = MIOLINK_REVA_TARGET_TDI_PIN,             /**< Target TDI (JTAG). */
	.tdo = MIOLINK_REVA_TARGET_TDO_PIN,             /**< Target TDO (JTAG). */
	.uart_tx = MIOLINK_REVA_TARGET_UART_TX_PIN,     /**< Target UART TX (probe → target). */
	.uart_rx = MIOLINK_REVA_TARGET_UART_RX_PIN,     /**< Target UART RX (target → probe). */
	.reset = MIOLINK_REVA_TARGET_NRST_PIN,         /**< Target nRESET / SRST output. */
	.reset_state = true,                            /**< Default nRESET level when idle (active-low semantics). */
};

static const platform_led_pins_t miolink_rev_a_led_pins = {
	.act = MIOLINK_REVA_LED_ACT_PIN, /**< Activity / general status LED. */
	.ser = MIOLINK_REVA_LED_SER_PIN, /**< UART / serial activity LED. */
	.err = MIOLINK_REVA_LED_ERR_PIN, /**< Error / fault LED. */
};

static const platform_vtref_info_t miolink_rev_a_vtref_info = {
	.enable_pin = MIOLINK_REVA_TARGET_VOLTAGE_ENABLE_PIN, /**< Target supply enable output. */
	.fault_pin = MIOLINK_REVA_TARGET_VOLTAGE_FAULT_PIN,   /**< Power-good or fault sense input. */
	.adc_channel = MIOLINK_REVA_TARGET_VOLTAGE_ADC_CHANNEL, /**< ADC channel index for rail monitoring. */
};

/**
 * \brief GPIO maps for MioLink main board — hardware revision B.
 *
 * Numeric values come from board headers (\c MIOLINK_REVB_*).
 */

static const platform_target_pins_t miolink_rev_b_target_pins = {
	.tck = MIOLINK_REVB_TARGET_TCK_PIN,             /**< Target SWD/JTAG clock (TCK). */
	.tms = MIOLINK_REVB_TARGET_TMS_PIN,             /**< Target TMS / bidirectional SWDIO data. */
	.tms_dir = MIOLINK_REVB_TARGET_TMS_DIR_PIN,     /**< TMS/SWDIO direction control (when used). */
	.tdi = MIOLINK_REVB_TARGET_TDI_PIN,             /**< Target TDI (JTAG). */
	.tdo = MIOLINK_REVB_TARGET_TDO_PIN,             /**< Target TDO (JTAG). */
	.uart_tx = MIOLINK_REVB_TARGET_UART_TX_PIN,     /**< Target UART TX (probe → target). */
	.uart_rx = MIOLINK_REVB_TARGET_UART_RX_PIN,     /**< Target UART RX (target → probe). */
	.reset = MIOLINK_REVB_TARGET_NRST_PIN,         /**< Target nRESET / SRST output. */
	.reset_state = true,                            /**< Default nRESET level when idle (active-low semantics). */
};

static const platform_led_pins_t miolink_rev_b_led_pins = {
	.act = MIOLINK_REVB_LED_ACT_PIN, /**< Activity / general status LED. */
	.ser = MIOLINK_REVB_LED_SER_PIN, /**< UART / serial activity LED. */
	.err = MIOLINK_REVB_LED_ERR_PIN, /**< Error / fault LED. */
};

static const platform_vtref_info_t miolink_rev_b_vtref_info = {
	.enable_pin = MIOLINK_REVB_TARGET_VOLTAGE_ENABLE_PIN, /**< Target supply enable output. */
	.fault_pin = MIOLINK_REVB_TARGET_VOLTAGE_FAULT_PIN,   /**< Power-good or fault sense input. */
	.adc_channel = MIOLINK_REVB_TARGET_VOLTAGE_ADC_CHANNEL, /**< ADC channel index for rail monitoring. */
};
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK_PICO

/**
 * \brief GPIO maps for MioLink_Pico carrier (\c MIOLINK_PICO_* board macros).
 */

static const platform_target_pins_t miolink_pico_target_pins = {
	.tck = MIOLINK_PICO_TARGET_TCK_PIN,             /**< Target SWD/JTAG clock (TCK). */
	.tms = MIOLINK_PICO_TARGET_TMS_PIN,             /**< Target TMS / bidirectional SWDIO data. */
	.tms_dir = MIOLINK_PICO_TARGET_TMS_DIR_PIN,     /**< TMS/SWDIO direction control (when used). */
	.tdi = MIOLINK_PICO_TARGET_TDI_PIN,             /**< Target TDI (JTAG). */
	.tdo = MIOLINK_PICO_TARGET_TDO_PIN,             /**< Target TDO (JTAG). */
	.uart_tx = MIOLINK_PICO_TARGET_UART_TX_PIN,     /**< Target UART TX (probe → target). */
	.uart_rx = MIOLINK_PICO_TARGET_UART_RX_PIN,     /**< Target UART RX (target → probe). */
	.reset = MIOLINK_PICO_TARGET_NRST_PIN,         /**< Target nRESET / SRST output. */
	.reset_state = true,                            /**< Default nRESET level when idle (active-low semantics). */
};

static const platform_led_pins_t miolink_pico_led_pins = {
	.act = MIOLINK_PICO_LED_ACT_PIN, /**< Activity / general status LED. */
	.ser = MIOLINK_PICO_LED_SER_PIN, /**< UART / serial activity LED. */
	.err = MIOLINK_PICO_LED_ERR_PIN, /**< Error / fault LED. */
};

static const platform_vtref_info_t miolink_pico_vtref_info = {
	.enable_pin = MIOLINK_PICO_TARGET_VOLTAGE_ENABLE_PIN, /**< Target supply enable output. */
	.fault_pin = MIOLINK_PICO_TARGET_VOLTAGE_FAULT_PIN,   /**< Power-good or fault sense input. */
	.adc_channel = MIOLINK_PICO_TARGET_VOLTAGE_ADC_CHANNEL, /**< ADC channel index for rail monitoring. */
};
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_BOARD

/**
 * \brief GPIO maps for Raspberry Pi Pico as a debug adapter (\c PICO_* board macros).
 *
 * \note No separate TMS direction line; unused LED slots use \ref PIN_NOT_CONNECTED.
 */

static const platform_target_pins_t pico_target_pins = {
	.tck = PICO_TARGET_TCK_PIN,                     /**< Target SWD/JTAG clock (TCK). */
	.tms = PICO_TARGET_TMS_PIN,                     /**< Target TMS / SWDIO. */
	.tms_dir = PIN_NOT_CONNECTED,                   /**< Not routed on this pinout. */
	.tdi = PICO_TARGET_TDI_PIN,                     /**< Target TDI (JTAG). */
	.tdo = PICO_TARGET_TDO_PIN,                     /**< Target TDO (JTAG). */
	.uart_tx = PICO_TARGET_UART_TX_PIN,             /**< Target UART TX (probe → target). */
	.uart_rx = PICO_TARGET_UART_RX_PIN,             /**< Target UART RX (target → probe). */
	.reset = PICO_TARGET_NRST_PIN,                 /**< Target nRESET / SRST output. */
	.reset_state = false,                           /**< Default nRESET level when idle. */
};

static const platform_led_pins_t pico_led_pins = {
	.act = PICO_LED_ACT_PIN,       /**< On-board LED used as activity. */
	.ser = PIN_NOT_CONNECTED,      /**< Not populated / not used. */
	.err = PIN_NOT_CONNECTED,      /**< Not populated / not used. */
};

#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_W_BOARD

/**
 * \brief GPIO maps for Raspberry Pi Pico W as a debug adapter (\c PICO_W_* board macros).
 *
 * \note On-board status LEDs are not wired for BMP-style act/ser/err; all three entries are
 *       \ref PIN_NOT_CONNECTED.
 */

static const platform_target_pins_t pico_w_target_pins = {
	.tck = PICO_W_TARGET_TCK_PIN,                   /**< Target SWD/JTAG clock (TCK). */
	.tms = PICO_W_TARGET_TMS_PIN,                   /**< Target TMS / SWDIO. */
	.tms_dir = PIN_NOT_CONNECTED,                   /**< Not routed on this pinout. */
	.tdi = PICO_W_TARGET_TDI_PIN,                   /**< Target TDI (JTAG). */
	.tdo = PICO_W_TARGET_TDO_PIN,                   /**< Target TDO (JTAG). */
	.uart_tx = PICO_W_TARGET_UART_TX_PIN,           /**< Target UART TX (probe → target). */
	.uart_rx = PICO_W_TARGET_UART_RX_PIN,           /**< Target UART RX (target → probe). */
	.reset = PICO_W_TARGET_NRST_PIN,               /**< Target nRESET / SRST output. */
	.reset_state = false,                           /**< Default nRESET level when idle. */
};

static const platform_led_pins_t pico_w_led_pins = {
	.act = PIN_NOT_CONNECTED, /**< No dedicated BMP activity GPIO (see block comment above). */
	.ser = PIN_NOT_CONNECTED, /**< Not used. */
	.err = PIN_NOT_CONNECTED, /**< Not used. */
};
#endif

platform_device_type_t platform_hwtype(void)
{
	return device_type;
}

void platform_update_hwtype(void)
{
#if PLATFORM_AUTO_DETECT
	if (device_type == PLATFORM_DEVICE_TYPE_NOT_SET) {
		const int hwversion = platform_hwversion();

		if (hwversion == HWVERSION_PICO) {
			gpio_init(PICO_W_DETECT_CYW43_CS_PIN);
			gpio_set_dir(PICO_W_DETECT_CYW43_CS_PIN, GPIO_OUT);
			gpio_put(PICO_W_DETECT_CYW43_CS_PIN, false);

			adc_init();
			adc_gpio_init(ADC_BASE_PIN + PICO_W_DETECT_ADC_CHANNEL);
			adc_select_input(PICO_W_DETECT_ADC_CHANNEL);

			/* Just drop the first measurement */
			adc_read();
			volatile uint16_t adc_result = adc_read();

			gpio_init(PICO_W_DETECT_CYW43_CS_PIN);
			gpio_init(ADC_BASE_PIN + PICO_W_DETECT_ADC_CHANNEL);

			if (adc_result < PICO_W_DETECT_ADC_THRESHOLD) {
				cyw43_arch_init();
				device_type = PLATFORM_DEVICE_TYPE_PICO_W;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_PICO;
			}
		} else {
			gpio_init(MIOLINK_TYPE_PIN_0);
			gpio_set_pulls(MIOLINK_TYPE_PIN_0, true, false);

			for (uint32_t i = 0; i < 100000; i++)
				__nop();

			if (gpio_get(MIOLINK_TYPE_PIN_0)) {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK_PICO;
			}
		}
	}
#elif PLATFORM_PICO_W_BOARD
	cyw43_arch_init();
#endif
}

int platform_hwversion(void)
{
	static int hwversion = PLATFORM_HWVERSION_UNKNOWN;

	if (hwversion == PLATFORM_HWVERSION_UNKNOWN) {
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
#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK
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
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK_PICO
	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_target_pins;
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_BOARD
	case PLATFORM_DEVICE_TYPE_PICO:
	    p_pins = &pico_target_pins;
	    break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_W_BOARD
	case PLATFORM_DEVICE_TYPE_PICO_W:
	    p_pins = &pico_w_target_pins;
	    break;
#endif
		

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
#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			p_pins = &miolink_rev_a_led_pins;
		} else {
			p_pins = &miolink_rev_b_led_pins;
		}
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK_PICO
	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_led_pins;
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_BOARD
	case PLATFORM_DEVICE_TYPE_PICO:
		p_pins = &pico_led_pins;
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_W_BOARD
	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_pins = &pico_w_led_pins;
		break;
#endif

	default:
		break;
	}

	return p_pins;
}

const platform_vtref_info_t *platform_get_vtref_info(void)
{
	const platform_vtref_info_t *p_vtref_info = NULL;

	switch (device_type) {
#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			p_vtref_info = &miolink_rev_a_vtref_info;
		} else {
			p_vtref_info = &miolink_rev_b_vtref_info;
		}
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK_PICO
	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_vtref_info = &miolink_pico_vtref_info;
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_BOARD
	case PLATFORM_DEVICE_TYPE_PICO:
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_PICO_W_BOARD
	case PLATFORM_DEVICE_TYPE_PICO_W:
		break;
#endif

	default:
		break;
	}

	return p_vtref_info;
}

void platform_make_board_ident(void)
{
	switch (platform_hwtype()) {
#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK
	case PLATFORM_DEVICE_TYPE_MIOLINK:
	    snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_MIOLINK, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK;
		break;
#endif

#if PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK_PICO
	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_MIOLINK_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK_PICO;
		break;
#endif

#if PLATFORM_AUTO_DETECT
	case PLATFORM_DEVICE_TYPE_PICO:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO;
		break;

	case PLATFORM_DEVICE_TYPE_PICO_W:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, PLATFORM_IDENT_PICO_W, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO_W;
		break;
#elif PLATFORM_PICO_BOARD
	case PLATFORM_DEVICE_TYPE_PICO:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, BOARD_IDENT_NON_MIOLINK, FIRMWARE_VERSION);
		platform_ident_ptr = BOARD_IDENT_NON_MIOLINK;
		break;
#elif PLATFORM_PICO_W_BOARD
	case PLATFORM_DEVICE_TYPE_PICO_W:
		snprintf(board_ident, sizeof(board_ident), BOARD_IDENT_FORMAT, BOARD_IDENT_NON_MIOLINK, FIRMWARE_VERSION);
		platform_ident_ptr = BOARD_IDENT_NON_MIOLINK;
		break;
#endif

	default:
		assert(false);
		break;
	}
}

const char *platform_ident(void)
{
	return platform_ident_ptr;
}