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

#ifndef MIOLINK_PLATFORM_H
#define MIOLINK_PLATFORM_H

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Global Definitions
 **********************************************************************************************************************/

#if defined(BOARD_AUTO)
#define PLATFORM_AUTO_DETECT 1 /**< Build targets runtime board detection */
#else
#define PLATFORM_AUTO_DETECT 0
#endif

#if defined(PICO_CYW43_SUPPORTED) && PICO_CYW43_SUPPORTED == 1
#define PLATFORM_WIFI_SUPPORTED 1 /**< Board has CYW43 (Pico W class) */
#else
#define PLATFORM_WIFI_SUPPORTED 0
#endif

#if defined(BOARD_MIOLINK)
#define PLATFORM_BOARD_MIOLINK 1 /**< MioLink board */
#else
#define PLATFORM_BOARD_MIOLINK 0
#endif

#if defined(BOARD_MIOLINK_PICO)
#define PLATFORM_BOARD_MIOLINK_PICO 1 /**< MioLink_Pico board */
#else
#define PLATFORM_BOARD_MIOLINK_PICO 0
#endif

#if defined(BOARD_AUTO) && (defined(BOARD_MIOLINK) || defined(BOARD_MIOLINK_PICO))
#error "Multiple boards defined. Please define only one of BOARD_AUTO, BOARD_MIOLINK and BOARD_MIOLINK_PICO."
#endif

#if defined(BOARD_MIOLINK) && defined(BOARD_MIOLINK_PICO)
#error "Multiple boards defined. Please define only one of BOARD_MIOLINK and BOARD_MIOLINK_PICO."
#endif

/**
 * \brief True when building for a generic Pico/Pico W without MioLink board macros 
 */
#define PLATFORM_IS_NON_MIOLINK_BOARD (!PLATFORM_AUTO_DETECT && !PLATFORM_BOARD_MIOLINK && !PLATFORM_BOARD_MIOLINK_PICO)

/**
 * \brief Plain RP2040 Pico (no CYW43)
 */
#define PLATFORM_PICO_BOARD           (PLATFORM_IS_NON_MIOLINK_BOARD && !PLATFORM_WIFI_SUPPORTED)

/**
 * \brief Pico W (CYW43 present)
 */
#define PLATFORM_PICO_W_BOARD         (PLATFORM_IS_NON_MIOLINK_BOARD && PLATFORM_WIFI_SUPPORTED)

#if PLATFORM_PICO_BOARD
#include "pico_pinout.h"
#elif PLATFORM_PICO_W_BOARD
#include "pico_w_pinout.h"
#endif

#define TARGET_SWD_IDLE_CYCLES (8) /**< Minimum idle SWD clock cycles between transactions */
#if (TARGET_SWD_IDLE_CYCLES < 8)
#error "TARGET_SWD_IDLE_CYCLES should be at least 8"
#endif

extern bool running_status; /**< BMP “target running” LED / morse hint state */

#define MACRO_VALUE_STR_WRAP(macro) #macro
#define MACRO_VALUE_STR(macro)      MACRO_VALUE_STR_WRAP(macro)

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp; /**< Verbose BMP logging when \c ENABLE_DEBUG */
#endif

#define PLATFORM_IDENT "(Unknown MioLink) " /**< Default USB string fragment before \c platform_make_board_ident() */

#define BOARD_IDENT_LENGTH (256U) /**< Max length for composite board identification string */

#define PLATFORM_MIOLINK
#define PLATFORM_MIOLINK_REV_A (1) /**< Hardware revision A discriminator */
#define PLATFORM_MIOLINK_REV_B (2) /**< Hardware revision B discriminator */

#define PLATFORM_HAS_TRACESWO          /**< Platform has traceswo support */
#define PLATFORM_HAS_CUSTOM_COMMANDS   /**< Platform has custom commands support */
/** TODO: Disable power switch feature for Pico boards */
#define PLATFORM_HAS_POWER_SWITCH      /**< Platform has power switch support */

#define GDB_ENDPOINT_NOTIF (0x84) /**< USB notification endpoint address for GDB CDC */
#define GDB_ENDPOINT       (0x01) /**< USB bulk endpoint address for GDB CDC */

#define SERIAL_ENDPOINT_NOTIF (0x85) /**< USB notification endpoint for target serial CDC */
#define SERIAL_ENDPOINT       (0x02) /**< USB bulk endpoint for target serial CDC */

#ifdef PLATFORM_HAS_TRACESWO
#define SWO_ENCODING_MANCHESTER (1)                 /**< Manchester encoding */
#define SWO_ENCODING_UART       (2)                 /**< UART encoding */
#define SWO_ENCODING            (SWO_ENCODING_UART) /**< Active SWO line code (NRZ UART on RP2040) */
#define SWO_ENDPOINT            (0x83)              /**< USB endpoint for decoded SWO stream */
#endif

#define PIN_NOT_CONNECTED (0xFF) /**< GPIO number sentinel for unpopulated pins */

#define MIOLINK_TYPE_PIN_0 (16) /**< Board ID: high = MioLink; low = MioLink_Pico */

#define HWVERSION_PIN_0 (15) /**< Hardware version strap bit 0 */
#define HWVERSION_PIN_1 (14) /**< Hardware version strap bit 1 */

#define HWVERSION_PICO (0x03 + 1) /**< Strap value when both version pins read as Pico-style */

#define SET_RUN_STATE(state)   running_status = (state)         /**< Target run-state hint for UI */
#define SET_IDLE_STATE(state)  platform_set_idle_state(state)   /**< Idle LED / morse path */
#define SET_ERROR_STATE(state) platform_set_error_state(state)  /**< Error LED path */

#define USB_SERIAL_UART_MAIN    (uart1) /**< Primary target UART instance. */
#define USB_SERIAL_UART_TDI_TDO (uart0) /**< Alternate UART on TDI/TDO when muxed */

#define USB_SERIAL_UART_MAIN_IRQ    (UART_IRQ_NUM(USB_SERIAL_UART_MAIN))
#define USB_SERIAL_UART_TDI_TDO_IRQ (UART_IRQ_NUM(USB_SERIAL_UART_TDI_TDO))
#define USB_SERIAL_TRACESWO_DMA_IRQ (DMA_IRQ_0) /**< DMA_IRQ_0: UART + SWO (dedicated channels). Vtref ADC uses DMA_IRQ_1. */

#define PLATFORM_PRIORITY_LOW    (tskIDLE_PRIORITY + 1)
#define PLATFORM_PRIORITY_NORMAL (tskIDLE_PRIORITY + 2)
#define PLATFORM_PRIORITY_HIGH   (tskIDLE_PRIORITY + 3)

/**********************************************************************************************************************
 * Global Types
 **********************************************************************************************************************/

/**
 * \brief Detected or configured product family for pinout and features.
 */
typedef enum {
	PLATFORM_DEVICE_TYPE_NOT_SET = 0,
	PLATFORM_DEVICE_TYPE_MIOLINK,
	PLATFORM_DEVICE_TYPE_MIOLINK_PICO,
	PLATFORM_DEVICE_TYPE_PICO,
	PLATFORM_DEVICE_TYPE_PICO_W,
} platform_device_type_t;

/**
 * \brief Target connector / probe signals (JTAG/SWD + UART + SRST).
 */
typedef struct {
	uint8_t tck;
	uint8_t tms;
	uint8_t tms_dir;
	uint8_t tdi;
	uint8_t tdo;
	uint8_t uart_tx;
	uint8_t uart_rx;
	uint8_t reset;
	bool reset_state;
} platform_target_pins_t;

/**
 * \brief Activity / error / serial status LED GPIO numbers.
 */
typedef struct {
	uint8_t act;
	uint8_t ser;
	uint8_t err;
} platform_led_pins_t;

/**
 * \brief Target power (VTref) enable, fault input, and ADC channel for monitoring.
 */
typedef struct {
	uint8_t enable_pin;
	uint8_t fault_pin;
	uint8_t adc_channel;
} platform_vtref_info_t;

/**********************************************************************************************************************
 * Global Functions Prototypes
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
 * \param state \c true to set the idle state, \c false to clear it
 */
void platform_set_idle_state(bool state);

/**
 * \brief Toggle idle LED (heartbeat / activity).
 */
void platform_toggle_idle_state(void);

/**
 * \brief Drive fault / error LED line.
 *
 * \param state \c true to set the error state, \c false to clear it
 */
void platform_set_error_state(bool state);

/**
 * \brief Drive “serial activity” LED.
 *
 * \param state \c true to set the serial state, \c false to clear it
 */
void platform_set_serial_state(bool state);

#endif /* MIOLINK_PLATFORM_H */
