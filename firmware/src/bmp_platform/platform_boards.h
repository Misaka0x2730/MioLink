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

/* This file provides MioLink platform identification and board-detection definitions. */

#ifndef MIOLINK_PLATFORM_BOARDS_H
#define MIOLINK_PLATFORM_BOARDS_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#if defined(BOARD_AUTO_RP2040)
#define PLATFORM_AUTO_DETECT (1) /**< Build targets runtime board detection. */
#else
#define PLATFORM_AUTO_DETECT (0) /**< Build targets runtime board detection. */
#endif

#if defined(PICO_CYW43_SUPPORTED) && (PICO_CYW43_SUPPORTED == 1)
#define PLATFORM_WIFI_SUPPORTED (1) /**< Board has CYW43 (Pico W class). */
#else
#define PLATFORM_WIFI_SUPPORTED (0) /**< Board has CYW43 (Pico W class). */
#endif

#if defined(BOARD_MIOLINK)
#define PLATFORM_BOARD_MIOLINK (1) /**< MioLink board. */
#else
#define PLATFORM_BOARD_MIOLINK (0) /**< MioLink board. */
#endif

#if defined(BOARD_MIOLINK_PICO)
#define PLATFORM_BOARD_MIOLINK_PICO (1) /**< MioLink_Pico board. */
#else
#define PLATFORM_BOARD_MIOLINK_PICO (0) /**< MioLink_Pico board. */
#endif

#if defined(BOARD_AUTO_RP2040) && (defined(BOARD_MIOLINK) || defined(BOARD_MIOLINK_PICO))
#error "Multiple boards defined. Please define only one of BOARD_AUTO_RP2040, BOARD_MIOLINK and BOARD_MIOLINK_PICO."
#endif

#if defined(BOARD_MIOLINK) && defined(BOARD_MIOLINK_PICO)
#error "Multiple boards defined. Please define only one of BOARD_MIOLINK and BOARD_MIOLINK_PICO."
#endif

/**
 * \brief True when building for any MioLink board.
 * */
#define PLATFORM_IS_MIOLINK_BOARD (PLATFORM_AUTO_DETECT || PLATFORM_BOARD_MIOLINK || PLATFORM_BOARD_MIOLINK_PICO)

/**
 * \brief True when building for a generic Pico/Pico W without MioLink board macros
 */
#define PLATFORM_IS_NON_MIOLINK_BOARD (!PLATFORM_IS_MIOLINK_BOARD)

/**
 * \brief Plain RP2040 Pico (no CYW43)
 */
#define PLATFORM_PICO_BOARD (PLATFORM_IS_NON_MIOLINK_BOARD && !PLATFORM_WIFI_SUPPORTED)

/**
 * \brief Pico W (CYW43 present)
 */
#define PLATFORM_PICO_W_BOARD (PLATFORM_IS_NON_MIOLINK_BOARD && PLATFORM_WIFI_SUPPORTED)

/* Platform-specific pinout headers for Pico and Pico W */
#if PLATFORM_PICO_BOARD
#include "pico_pinout.h"
#elif PLATFORM_PICO_W_BOARD
#include "pico_w_pinout.h"
#endif

#define PLATFORM_IDENT "(Unknown MioLink) " /**< Default USB string fragment before \c platform_make_board_ident() */

#define BOARD_IDENT_LENGTH (256U) /**< Max length for composite board identification string */

#define PLATFORM_MIOLINK_REV_A (1) /**< Hardware revision A discriminator. */
#define PLATFORM_MIOLINK_REV_B (2) /**< Hardware revision B discriminator. */

#define MIOLINK_TYPE_PIN_0 (16) /**< Board ID: high = MioLink; low = MioLink_Pico */

#define HWVERSION_PIN_0 (15) /**< Hardware version strap bit 0 */
#define HWVERSION_PIN_1 (14) /**< Hardware version strap bit 1 */

#define HWVERSION_PICO (0x03 + 1) /**< Strap value when both version pins read as Pico-style */

/**********************************************************************************************************************
 * Public Types
 **********************************************************************************************************************/

/**
 * \brief Detected or configured product family for pinout and features.
 */
typedef enum {
    PLATFORM_DEVICE_TYPE_NOT_SET = 0,  /**< Detection has not run yet; \c platform_update_hwtype must be called. */
    PLATFORM_DEVICE_TYPE_MIOLINK,      /**< MioLink main board (rev A or rev B). */
    PLATFORM_DEVICE_TYPE_MIOLINK_PICO, /**< MioLink_Pico carrier. */
    PLATFORM_DEVICE_TYPE_PICO,         /**< Raspberry Pi Pico used as a debug probe. */
    PLATFORM_DEVICE_TYPE_PICO_W,       /**< Raspberry Pi Pico W used as a debug probe. */
} platform_device_type_t;

#endif /* MIOLINK_PLATFORM_BOARDS_H */
