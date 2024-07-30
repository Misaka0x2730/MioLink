/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
 * Modified 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 * Modified by Rachel Mant <git@dragonmux.network>
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

#include "pico/time.h"

#include "FreeRTOS.h"
#include "timers.h"

bool running_status = false;
static volatile uint32_t time_ms = 0;
uint32_t target_clk_divider = 20;

static size_t morse_tick = 0;
#if defined(PLATFORM_HAS_POWER_SWITCH)
static uint8_t monitor_ticks = 0;
static uint8_t monitor_error_count = 0;
#endif

static void __not_in_flash_func(sys_timer_callback)(TimerHandle_t xTimer)
{
    time_ms += SYSTICKMS;
    if (morse_tick >= MORSECNT)
    {
        if (running_status)
        {
#if defined(LED_ACT_PIN)
            gpio_xor_mask(1 << LED_ACT_PIN);
#endif
        }
        //TODO: Update this
        //usb_config_morse_msg_update();
        SET_ERROR_STATE(morse_update());
        morse_tick = 0;
    }
    else
    {
        ++morse_tick;
    }

#if defined(PLATFORM_MIOLINK)
#if defined(PLATFORM_HAS_POWER_SWITCH)
    /* First check if target power is presently enabled */
    if (platform_target_get_power())
    {
        /* If we're on the 3rd tick (30 ms), check the power fault pin */
        if (++monitor_ticks == 3)
        {
            monitor_ticks = 0;

            /* Now compare the reference against the known good range */
            if (platform_target_is_power_ok() == false)
            {
                monitor_error_count++;
            }
            else if (monitor_error_count)
            {
                monitor_error_count--;
            }

            /* Something's wrong, and it is not a glitch, so turn tpwr off and set the morse blink pattern */
            if (monitor_error_count > 2)
            {
                monitor_error_count = 0;

                platform_target_set_power(false);
                morse("TPWR ERROR", true);
            }
        }
    }
    else
    {
        monitor_ticks = 0;
    }
#endif
#endif
}

void platform_timing_init(void)
{
    TimerHandle_t system_timer = xTimerCreate("system", pdMS_TO_TICKS(SYSTICKMS), pdTRUE, NULL, sys_timer_callback);
    xTimerStart(system_timer, 0);
}

void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}

void platform_max_frequency_set(uint32_t freq)
{
    (void)freq;
}

uint32_t platform_max_frequency_get(void)
{
    return FREQ_FIXED;
}