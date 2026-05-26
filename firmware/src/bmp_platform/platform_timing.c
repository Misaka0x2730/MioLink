/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2023 1BitSquared <info@1bitsquared.com>
 * Modified by Rachel Mant <git@dragonmux.network>
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

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "platform.h"
#include "platform_timing.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "tap_pio.h"
#include "usb.h"
#include "morse.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define TPWR_CHECK_PERIOD_TICKS (3) /**< TPWR / VTref health check period, in system ticks. */
#define TPWR_FAULT_STREAK_LIMIT (5) /**< Consecutive VTref fault observations required to trip TPWR shutdown. */

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

bool running_status = false; /**< Whether a target is currently being driven by GDB. */
uint32_t target_interface_frequency = PLATFORM_DEFAULT_FREQUENCY; /**< Cached effective TAP/SWD clock (Hz). */

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

static volatile uint32_t time_ms = 0; /**< Free-running millisecond counter, advanced by the system tick timer. */

static size_t morse_tick = 0; /**< Tick counter to time morse status updates. */

#if defined(PLATFORM_HAS_POWER_SWITCH)
/**
 * \brief Sub-tick counter pacing the periodic VTref health check; wraps at \c TPWR_CHECK_PERIOD_TICKS.
 */
static uint8_t tpwr_check_period_ticks = 0;

/**
 * \brief Consecutive VTref fault observations; triggers TPWR shutdown when above \c TPWR_FAULT_STREAK_LIMIT.
 */
static uint8_t tpwr_consecutive_faults = 0;
#endif

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Pull pending USB configuration changes and emit / clear matching morse status messages.
 */
static void usb_config_morse_msg_update(void);

/**
 * \brief FreeRTOS software-timer callback: advances \c time_ms and drives morse / VTref monitor.
 *
 * \param[in] xTimer Timer handle that fired (unused).
 */
static void timing_application_timer_cb(TimerHandle_t xTimer);

/**
 * \brief Compute the peripheral clock used to derive PIO state-machine dividers (\c clk_sys / 8).
 *
 * \return Peripheral clock frequency in Hz.
 */
static uint32_t platform_get_interface_periph_clk(void);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static void usb_config_morse_msg_update(void)
{
    if (usb_config_is_updated()) {
        if (usb_get_config() != USB_CONFIG_STATE_CONFIGURED) {
            morse("NO USB HOST.", true);
        } else {
            morse(NULL, false);
        }

        usb_config_clear_updated();
    }
}

static void timing_application_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

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

#if defined(PLATFORM_HAS_POWER_SWITCH)
    /* First check if target power is presently enabled */
    if (platform_target_get_power()) {
        /* If we're on the 3rd tick (30 ms), check the power fault pin */
        if (++tpwr_check_period_ticks == TPWR_CHECK_PERIOD_TICKS) {
            tpwr_check_period_ticks = 0;

            /* Now compare the reference against the known good range */
            if (platform_target_is_power_ok() == false) {
                tpwr_consecutive_faults++;
            } else if (tpwr_consecutive_faults) {
                tpwr_consecutive_faults--;
            }

            /* Something's wrong, and it is not a glitch, so turn tpwr off and set the morse blink pattern */
            if (tpwr_consecutive_faults >= TPWR_FAULT_STREAK_LIMIT) {
                tpwr_consecutive_faults = 0;

                platform_target_set_power(false);
                morse("TPWR ERROR", true);
            }
        }
    } else {
        tpwr_check_period_ticks = 0;
    }
#endif
}

static uint32_t platform_get_interface_periph_clk(void)
{
    return clock_get_hz(clk_sys) / 8;
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

void platform_timing_init(void)
{
    TimerHandle_t application_timer =
        xTimerCreate("app_timer", pdMS_TO_TICKS(SYSTICKMS), true, NULL, timing_application_timer_cb);
    assert(application_timer != NULL);
    xTimerStart(application_timer, 0);
}

/**
 * \brief Block the calling task for the requested number of milliseconds.
 *
 * \param[in] ms Number of milliseconds to wait.
 */
void platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * \brief Current free-running millisecond counter value.
 *
 * \return Milliseconds since boot.
 */
uint32_t platform_time_ms(void)
{
    return time_ms;
}

/**
 * \brief Program the SWD and JTAG PIO state machines to the closest divider for \a freq.
 *
 * \param[in] freq Requested interface frequency in Hz.
 */
void platform_max_frequency_set(uint32_t freq)
{
    for (uint32_t i = 0; i < NUM_PIO_STATE_MACHINES; i++) {
        tap_pio_set_sm_freq(TAP_PIO_SWD, i, freq, platform_get_interface_periph_clk());
    }

    for (uint32_t i = 0; i < NUM_PIO_STATE_MACHINES; i++) {
        target_interface_frequency = tap_pio_set_sm_freq(TAP_PIO_JTAG, i, freq, platform_get_interface_periph_clk());
    }
}

/**
 * \brief Effective interface frequency that the JTAG/SWD PIO last accepted.
 *
 * \return Frequency in Hz.
 */
uint32_t platform_max_frequency_get(void)
{
    return target_interface_frequency;
}

uint32_t platform_timeout_time_left(const platform_timeout_s *const timeout)
{
    /* Wrap-safe if timeout intervals are below 2^31 ms (~24.8 days). */
    const int32_t left = (int32_t)(timeout->time - platform_time_ms());
    return (left > 0) ? (uint32_t)left : 0U;
}
