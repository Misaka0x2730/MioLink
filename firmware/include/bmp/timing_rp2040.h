/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
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

#ifndef PLATFORMS_TIMING_RP2040_H
#define PLATFORMS_TIMING_RP2040_H

#include <stdint.h>
#include <stdbool.h>

#include "platform_support.h"

#define PLATFORM_DEFAULT_FREQUENCY (4000000UL)

extern bool running_status;

void platform_timing_init(void);
uint32_t platform_timeout_time_left(const platform_timeout_s *target);

#endif /* PLATFORMS_TIMING_RP2040_H */