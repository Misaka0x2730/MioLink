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
#include "platform.h"

#include "hardware/gpio.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"

#include "platform_timing.h"
#include "serialno.h"

static bool idle_state = false;

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

	assert(false);
}

void platform_set_idle_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	if (p_pins != NULL) {
		if (p_pins->act != PIN_NOT_CONNECTED) {
			gpio_put(p_pins->act, state);
		} else if (platform_hwtype() == PLATFORM_DEVICE_TYPE_PICO_W) {
			if (idle_state != state) {
				cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
			}
		}
	}

	idle_state = state;
}

void platform_toggle_idle_state(void)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	idle_state = !idle_state;

	if (p_pins != NULL) {
		if (p_pins->act != PIN_NOT_CONNECTED) {
			gpio_xor_mask(1UL << p_pins->act);
		} else if (platform_hwtype() == PLATFORM_DEVICE_TYPE_PICO_W) {
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, idle_state);
		}
	}
}

void platform_set_error_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();

	if ((p_pins != NULL) && (p_pins->err != PIN_NOT_CONNECTED)) {
		gpio_put(p_pins->err, state);
	}
}

void platform_set_serial_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();

	if ((p_pins != NULL) && (p_pins->ser != PIN_NOT_CONNECTED)) {
		gpio_put(p_pins->ser, state);
	}
}

void platform_init(void)
{
	platform_update_hwtype();
	assert(platform_hwtype() != PLATFORM_DEVICE_TYPE_NOT_SET);

	platform_make_board_ident();
	read_serial_number();

	const platform_led_pins_t *led_pins = platform_get_led_pins();
	assert(led_pins != NULL);

	if (led_pins->act != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->act);
		gpio_set_dir(led_pins->act, GPIO_OUT);
		gpio_put(led_pins->act, false);
	}

	if (led_pins->ser != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->ser);
		gpio_set_dir(led_pins->ser, GPIO_OUT);
		gpio_put(led_pins->ser, false);
	}

	if (led_pins->err != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->err);
		gpio_set_dir(led_pins->err, GPIO_OUT);
		gpio_put(led_pins->err, false);
	}

	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		gpio_init(target_pins->reset);
		gpio_set_dir(target_pins->reset, GPIO_OUT);
		gpio_put(target_pins->reset, !target_pins->reset_state);
	}

	platform_vtref_init();
	platform_timing_init();
}

void platform_nrst_set_val(bool assert)
{
	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		if (assert) {
			gpio_put(target_pins->reset, target_pins->reset_state);
			platform_delay(10);
		} else {
			gpio_put(target_pins->reset, !target_pins->reset_state);
		}
	}
}

bool platform_nrst_get_val(void)
{
	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		return gpio_get(target_pins->reset) == target_pins->reset_state;
	}

	return false;
}

void platform_target_clk_output_enable(bool enable)
{
	(void)enable;
}

bool platform_spi_init(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	(void)device_select;
	return false;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	(void)bus;
	return value;
}

#if ENABLE_SYSVIEW_TRACE
uint32_t SEGGER_SYSVIEW_X_GetTimestamp(void)
{
	return time_us_32();
}
#endif

void tud_dfu_runtime_reboot_to_dfu_cb(void)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	uint32_t pin_mask = 0;

	if (p_pins != NULL) {
		if (p_pins->err == PIN_NOT_CONNECTED) {
			if (p_pins->act != PIN_NOT_CONNECTED) {
				pin_mask |= (1UL << p_pins->act);
			}
		} else {
			pin_mask |= (1UL << p_pins->err);
		}
	}
	/* Error pin is used as boot activity pin, enable all boot modes */
	reset_usb_boot(pin_mask, 0);
}