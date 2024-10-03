/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
 * Modified by Rachel Mant <git@dragonmux.network>
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
#include "morse.h"
#include "timing_rp2040.h"
#include "usb.h"

#include "pico/time.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "tap_pio_common.h"

bool running_status = false;
static volatile uint32_t time_ms = 0;
uint32_t target_clk_divider = UINT32_MAX;

static size_t morse_tick = 0;
static uint8_t monitor_ticks = 0;
static uint8_t monitor_error_count = 0;

uint32_t target_interface_frequency = PLATFORM_DEFAULT_FREQUENCY;

static void usb_config_morse_msg_update(void)
{
	if (usb_config_is_updated()) {
		if (usb_get_config() == 0)
			morse("NO USB HOST.", true);
		else
			morse(NULL, false);

		usb_config_clear_updated();
	}
}

static void timing_application_timer_cb(TimerHandle_t xTimer)
{
	time_ms += SYSTICKMS;
	if (morse_tick >= MORSECNT) {
		if (running_status) {
			platform_toggle_idle_state();
		}
		usb_config_morse_msg_update();
		SET_ERROR_STATE(morse_update());
		morse_tick = 0;
	} else {
		++morse_tick;
	}

	/* First check if target power is presently enabled */
	if (platform_target_get_power()) {
		/* If we're on the 3rd tick (30 ms), check the power fault pin */
		if (++monitor_ticks == 3) {
			monitor_ticks = 0;

			/* Now compare the reference against the known good range */
			if (platform_target_is_power_ok() == false) {
				monitor_error_count++;
			} else if (monitor_error_count) {
				monitor_error_count--;
			}

			/* Something's wrong, and it is not a glitch, so turn tpwr off and set the morse blink pattern */
			if (monitor_error_count > 5) {
				monitor_error_count = 0;

				platform_target_set_power(false);
				morse("TPWR ERROR", true);
			}
		}
	} else {
		monitor_ticks = 0;
	}
}

void platform_timing_init(void)
{
	TimerHandle_t application_timer =
		xTimerCreate("app_timer", pdMS_TO_TICKS(SYSTICKMS), true, NULL, timing_application_timer_cb);
	assert(application_timer != NULL);
	xTimerStart(application_timer, 0);
}

void platform_delay(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}

static uint32_t platform_get_max_interface_freq(void)
{
	const uint32_t sys_freq = clock_get_hz(clk_sys);
	uint32_t interface_freq = 0;

	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		if (platform_hwversion() == PLATFORM_MIOLINK_REV_A) {
			interface_freq = sys_freq / 10;
		} else {
			interface_freq = sys_freq / 8;
		}
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		interface_freq = sys_freq / 8;
		break;

	default:
		interface_freq = sys_freq / 10;
		break;
	}

	return interface_freq;
}

void platform_max_frequency_set(uint32_t freq)
{
	const uint32_t interface_freq = platform_get_max_interface_freq();

	const uint32_t min_freq = (interface_freq >> 16); /* Max divider = 65536 */
	const uint32_t max_freq = interface_freq;

	if (freq < min_freq) {
		freq = min_freq;
	} else if (freq > max_freq) {
		freq = max_freq;
	}

	uint32_t clkdiv_int = (interface_freq / freq);
	uint32_t clkdiv_fraq = ((((uint64_t)interface_freq) << 8) / freq) & 0xFF;

	if (clkdiv_int >= (((uint32_t)UINT16_MAX) + 1)) {
		clkdiv_int = 0;
		clkdiv_fraq = 0;
	}
	pio_sm_set_clkdiv_int_frac(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_SEQ);

	pio_sm_set_clkdiv_int_frac(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_SWD_PIO, TARGET_SWD_PIO_SM_ADIV5);

	pio_sm_set_clkdiv_int_frac(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_NEXT_CYCLE);

	pio_sm_set_clkdiv_int_frac(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TMS_SEQ);

	pio_sm_set_clkdiv_int_frac(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_TDO_SEQ);

	pio_sm_set_clkdiv_int_frac(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ, clkdiv_int, clkdiv_fraq);
	pio_sm_clkdiv_restart(TARGET_JTAG_PIO, TARGET_JTAG_PIO_SM_TDI_SEQ);

	target_interface_frequency = freq;
}

uint32_t platform_max_frequency_get(void)
{
	const uint32_t interface_freq = platform_get_max_interface_freq();

	uint32_t clkdiv_int =
		(TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM_SEQ].clkdiv & PIO_SM0_CLKDIV_INT_BITS) >> PIO_SM0_CLKDIV_INT_LSB;
	uint32_t clkdiv_frac =
		(TARGET_SWD_PIO->sm[TARGET_SWD_PIO_SM_SEQ].clkdiv & PIO_SM0_CLKDIV_FRAC_BITS) >> PIO_SM0_CLKDIV_FRAC_LSB;

	if (clkdiv_int == 0) {
		clkdiv_int = ((uint32_t)UINT16_MAX) + 1;
		clkdiv_frac = 0;
	}

	target_interface_frequency =
		(uint32_t)(((((uint64_t)interface_freq) << 8) / ((clkdiv_int << 8) + clkdiv_frac)) & 0xFFFFFFFF);

	return target_interface_frequency;
}

uint32_t platform_timeout_time_left(const platform_timeout_s *const t)
{
	/* Cache the current time for the whole calculation */
	const uint32_t counter = platform_time_ms();

	if ((counter & UINT32_C(0x80000000)) && !(t->time & UINT32_C(0x80000000))) {
		return UINT32_MAX - counter + t->time + 1;
	} else if (t->time > counter) {
		return t->time - counter;
	} else {
		return 0;
	}
}