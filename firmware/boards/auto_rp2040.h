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

/**
 * \brief Feature flag marking an RP2040 build that performs runtime board auto-detection.
 *
 * Defined when \c PICO_BOARD=auto_rp2040 selects this header. Consumers gate auto-detect code
 * paths (MioLink rev A/B vs MioLink_Pico vs Pico vs Pico W) on its presence.
 */
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

#define PICO_VSYS_PIN                  (29) /**< GPIO wired to VSYS divider on Pico/Pico W */

#define PICO_W_DETECT_CYW43_CS_PIN (CYW43_DEFAULT_PIN_WL_CS) /**< GPIO sampled to discriminate Pico vs Pico W. */

/**
 * \brief ADC threshold (12-bit, 3.3 V reference) for inferring CYW43 presence on \ref PICO_VSYS_PIN.
 *
 * Detection drives \c WL_CS (\ref PICO_W_DETECT_CYW43_CS_PIN) LOW and samples GPIO29 with the
 * RP2040 internal pull-up active (~50–80 kΩ to 3.3 V). What separates the board variants at
 * GPIO29:
 *   - **Pico W (CYW43 present):** GPIO29 carries the CYW43 \c WL_CLK net with a 10 kΩ pull-down
 *     to GND. That pull-down dominates the weak internal pull-up, holding GPIO29 well below 1 V
 *     (ADC well below 0x600).
 *   - **Genuine Pico:** the FET that gates the 200 kΩ / 100 kΩ VSYS divider stays OFF while the
 *     GPIO29 pull-up is active (the pulled-up GPIO29 keeps the FET back-biased), so the divider
 *     is disconnected from GPIO29. Only the internal pull-up acts, with a small bleed through
 *     the FET body diode into the divider midpoint, so GPIO29 settles a few hundred mV below
 *     the 3.3 V rail (ADC well above 0x600).
 *   - **Clones / non-Pico boards without the divider+FET:** GPIO29 sees only the internal
 *     pull-up and sits close to the 3.3 V rail (ADC well above 0x600). Treated as Pico (no
 *     CYW43 init).
 *
 * Exact ADC counts drift with chip-to-chip pull-up tolerance, VSYS, supply rail, and ADC noise,
 * but the two clusters stay separated by ~2 V. 0x600 (1536, ~1.24 V) sits in the middle of that
 * gap. Below threshold → Pico W; at or above → Pico/clone.
 */
#define PICO_W_DETECT_ADC_THRESHOLD (0x600)

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
