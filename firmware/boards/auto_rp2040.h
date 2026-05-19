/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2026 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#ifndef MIOLINK_BOARDS_AUTO_RP2040_H
#define MIOLINK_BOARDS_AUTO_RP2040_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "pinout/miolink_rev_a_pinout.h"
#include "pinout/miolink_rev_b_pinout.h"
#include "pinout/miolink_pico_pinout.h"
#include "pinout/pico_pinout.h"
#include "pinout/pico_w_pinout.h"

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

pico_board_cmake_set(PICO_PLATFORM, rp2040)
pico_board_cmake_set(PICO_CYW43_SUPPORTED, 1)

#define BOARD_AUTO_RP2040

/* CYW43 static pin map, kept here because PICO_BOARD=auto_rp2040 does not pull in the SDK
 * pico_w.h board header (which is where the SDK would normally provide these as defaults). */
#define CYW43_PIN_WL_DYNAMIC (0) /**< Disables dynamic CYW43 pin remapping. */

#define CYW43_DEFAULT_PIN_WL_REG_ON    (23) /**< CYW43 regulator enable pin. */
#define CYW43_DEFAULT_PIN_WL_DATA_OUT  (24) /**< CYW43 SPI MOSI / data out pin. */
#define CYW43_DEFAULT_PIN_WL_DATA_IN   (24) /**< CYW43 SPI MISO / data in pin (shared with data out). */
#define CYW43_DEFAULT_PIN_WL_HOST_WAKE (24) /**< CYW43 host-wake input pin. */
#define CYW43_DEFAULT_PIN_WL_CLOCK     (29) /**< CYW43 SPI clock pin. */
#define CYW43_DEFAULT_PIN_WL_CS        (25) /**< CYW43 SPI chip-select pin. */
#define CYW43_PIN_WL_HOST_WAKE         (24) /**< Active CYW43 host-wake pin override. */
#define CYW43_PIN_WL_REG_ON            (23) /**< Active CYW43 regulator-enable override. */
#define CYW43_WL_GPIO_COUNT            (3)  /**< Number of CYW43-managed virtual GPIOs. */
#define CYW43_WL_GPIO_LED_PIN          (0)  /**< CYW43 virtual GPIO that drives the on-board LED. */

#define PICO_VSYS_PIN               (29) /**< GPIO wired to VSYS divider on Pico/Pico W */

#define PICO_W_DETECT_CYW43_CS_PIN  (CYW43_DEFAULT_PIN_WL_CS)      /**< GPIO sampled to discriminate Pico vs Pico W. */
#define PICO_W_DETECT_ADC_CHANNEL   (PICO_VSYS_PIN - ADC_BASE_PIN) /**< ADC channel used for Pico W detection. */
#define PICO_W_DETECT_ADC_THRESHOLD (0x600)                        /**< ADC threshold for inferring CYW43 presence. */

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 (1) /**< Selects W25Q080-compatible boot stage 2. */

#if !defined(PICO_FLASH_SPI_CLKDIV)
#define PICO_FLASH_SPI_CLKDIV (2) /**< Flash SPI clock divider. */
#endif

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (2 * 1024 * 1024))

#if !defined(PICO_FLASH_SIZE_BYTES)
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024) /**< Default flash size in bytes. */
#endif

#if !defined(PICO_RP2040_B0_SUPPORTED)
#define PICO_RP2040_B0_SUPPORTED (0) /**< Disables RP2040 B0 silicon workarounds; all supported boards have B1+. */
#endif

#endif /* MIOLINK_BOARDS_AUTO_RP2040_H */
