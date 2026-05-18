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

#ifndef MIOLINK_BOARDS_MIOLINK_PICO_H
#define MIOLINK_BOARDS_MIOLINK_PICO_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "pinout/miolink_pico_pinout.h"

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

pico_board_cmake_set(PICO_PLATFORM, rp2040)

#define BOARD_MIOLINK_PICO

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

#endif /* MIOLINK_BOARDS_MIOLINK_PICO_H */
