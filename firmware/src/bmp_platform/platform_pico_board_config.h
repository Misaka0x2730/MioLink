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

/* Lightweight entry point that pulls in the Pico SDK board header (selected by PICO_BOARD)
 * so the BOARD_* feature flags it defines become visible to consumers regardless of the
 * caller's include order.
 *
 * The board header uses pico_board_cmake_set / pico_board_cmake_set_default markers that
 * carry meaning only for the Pico SDK's CMake board scanner; in C they must compile to no-ops.
 * pico.h would define them as empty for us, but it also pulls in pico/platform.h →
 * addressmap.h, which defines symbols like SRAM_BASE that clash with Black Magic target
 * headers (efm32, gd32, ...). Including pico/config_autogen.h directly, with the markers
 * stubbed locally, gives us only the active board header plus a CMSIS exception-rename
 * helper — no addressmap.h.
 *
 * If pico.h is later included in the same translation unit (or any other), its identical
 * empty definitions of the same markers are allowed redefinitions (same body), so there is
 * no -Wmacro-redefined warning.
 */

#ifndef MIOLINK_PLATFORM_PICO_BOARD_CONFIG_H
#define MIOLINK_PLATFORM_PICO_BOARD_CONFIG_H

#define pico_board_cmake_set(x, y)
#define pico_board_cmake_set_default(x, y)

#include "pico/config_autogen.h"

#endif /* MIOLINK_PLATFORM_PICO_BOARD_CONFIG_H */
