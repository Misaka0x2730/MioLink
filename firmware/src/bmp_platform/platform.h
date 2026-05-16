/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
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

#ifndef MIOLINK_PLATFORM_H
#define MIOLINK_PLATFORM_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "platform_boards.h"
#include "platform_periph.h"
#include "platform_threads.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define TARGET_SWD_IDLE_CYCLES (8) /**< Minimum idle SWD clock cycles between transactions */
#if (TARGET_SWD_IDLE_CYCLES < 8)
#error "TARGET_SWD_IDLE_CYCLES should be at least 8"
#endif

#define MACRO_VALUE_STR_WRAP(macro) #macro
#define MACRO_VALUE_STR(macro)      MACRO_VALUE_STR_WRAP(macro)

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
#endif

#define PLATFORM_HAS_TRACESWO        /**< Platform has traceswo support. */
#define PLATFORM_HAS_CUSTOM_COMMANDS /**< Platform has custom commands support. */

#if PLATFORM_IS_MIOLINK_BOARD
#define PLATFORM_HAS_POWER_SWITCH /**< Platform has power switch support */
#endif

#define GDB_ENDPOINT_NOTIF (0x84) /**< USB notification endpoint address for GDB CDC */
#define GDB_ENDPOINT       (0x01) /**< USB bulk endpoint address for GDB CDC */

#define SERIAL_ENDPOINT_NOTIF (0x85) /**< USB notification endpoint for target serial CDC */
#define SERIAL_ENDPOINT       (0x02) /**< USB bulk endpoint for target serial CDC */

#if defined(PLATFORM_HAS_TRACESWO)
#define SWO_ENCODING_MANCHESTER (1)                 /**< Manchester encoding */
#define SWO_ENCODING_UART       (2)                 /**< UART encoding */
#define SWO_ENCODING            (SWO_ENCODING_UART) /**< Active SWO line code (NRZ UART on RP2040) */
#define SWO_ENDPOINT            (0x83)              /**< USB endpoint for decoded SWO stream */
#endif

#define SET_RUN_STATE(state)   running_status = (state)          /**< Target run-state hint for UI. */
#define SET_IDLE_STATE(state)  platform_set_idle_state((state))  /**< Idle LED / morse path. */
#define SET_ERROR_STATE(state) platform_set_error_state((state)) /**< Error LED path. */

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief Target connector / probe signals (JTAG/SWD + UART + SRST).
 */
typedef struct {
    uint8_t tck;      /**< SWCLK / JTAG TCK GPIO. */
    uint8_t tms;      /**< SWDIO / JTAG TMS GPIO. */
    uint8_t tms_dir;  /**< Direction control for level-shifted TMS/SWDIO; \ref PIN_NOT_CONNECTED when absent. */
    uint8_t tdi;      /**< JTAG TDI GPIO. */
    uint8_t tdo;      /**< JTAG TDO / SWO input GPIO. */
    uint8_t uart_tx;  /**< Target UART TX GPIO. */
    uint8_t uart_rx;  /**< Target UART RX GPIO. */
    uint8_t reset;    /**< Target reset (nRST) GPIO; \ref PIN_NOT_CONNECTED when absent. */
    bool reset_state; /**< Level driven on \c reset to assert it (\c true = drive high to reset). */
} platform_target_pins_t;

/**
 * \brief Activity / error / serial status LED GPIO numbers.
 */
typedef struct {
    uint8_t act; /**< Activity / idle LED GPIO; \ref PIN_NOT_CONNECTED when absent. */
    uint8_t ser; /**< Serial-activity LED GPIO; \ref PIN_NOT_CONNECTED when absent. */
    uint8_t err; /**< Error / fault LED GPIO; \ref PIN_NOT_CONNECTED when absent. */
} platform_led_pins_t;

/**
 * \brief Target power (VTref) enable, fault input, and ADC channel for monitoring.
 */
typedef struct {
    uint8_t enable_pin;  /**< GPIO enabling the target power switch. */
    uint8_t fault_pin;   /**< GPIO that reports a power-switch fault (active level is board-specific). */
    uint8_t adc_channel; /**< ADC channel sampling the target VTref divider. */
} platform_vtref_info_t;

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

extern bool running_status; /**< BMP "target running" LED / morse hint state. */

#if ENABLE_DEBUG == 1
extern bool debug_bmp; /**< Verbose BMP logging when \c ENABLE_DEBUG is on. */
#endif

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Whether target power and fault lines report a healthy supply.
 *
 * \return \c true if the target power and fault lines report a healthy supply, \c false otherwise
 */
bool platform_target_is_power_ok(void);

/**
 * \brief Re-read system clock and refresh timing-dependent probe settings.
 */
void platform_update_sys_freq(void);

/**
 * \brief Cached device type from board ID GPIOs / compile-time selection.
 */
platform_device_type_t platform_hwtype(void);

/**
 * \brief Re-run hardware detection and update \c platform_hwtype() state.
 */
void platform_update_hwtype(void);

/**
 * \brief Active target pin map for the running board type.
 *
 * \return Pointer to the active target pin map
 */
const platform_target_pins_t *platform_get_target_pins(void);

/**
 * \brief LED pin map for the running board type.
 *
 * \return Pointer to the LED pin map
 */
const platform_led_pins_t *platform_get_led_pins(void);

/**
 * \brief VTref control pins and ADC channel, or \c NULL if the board has no monitor circuit.
 *
 * \return Pointer to the VTref control pins and ADC channel, or \c NULL if the board has no monitor circuit
 */
const platform_vtref_info_t *platform_get_vtref_info(void);

/**
 * \brief Fill USB descriptor / ident string with board name and revision.
 */
void platform_make_board_ident(void);

/**
 * \brief Start ADC+DMA average monitor for target rail voltage (when \c platform_get_vtref_info() is non-NULL).
 */
void platform_vtref_init(void);

/**
 * \brief Drive idle (not actively talking to GDB) LED indication.
 *
 * \param[in] state \c true to set the idle state, \c false to clear it
 */
void platform_set_idle_state(bool state);

/**
 * \brief Toggle idle LED (heartbeat / activity).
 */
void platform_toggle_idle_state(void);

/**
 * \brief Drive fault / error LED line.
 *
 * \param[in] state \c true to set the error state, \c false to clear it
 */
void platform_set_error_state(bool state);

/**
 * \brief Drive “serial activity” LED.
 *
 * \param[in] state \c true to set the serial state, \c false to clear it
 */
void platform_set_serial_state(bool state);

#endif /* MIOLINK_PLATFORM_H */
