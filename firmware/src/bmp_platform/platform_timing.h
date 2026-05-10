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

#ifndef MIOLINK_PLATFORM_TIMING_H
#define MIOLINK_PLATFORM_TIMING_H

/**********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/

#include "general.h"

/**********************************************************************************************************************
 * Global Definitions
 **********************************************************************************************************************/

/** \brief Default SWD/JTAG interface frequency (Hz) when nothing else is configured. */
#define PLATFORM_DEFAULT_FREQUENCY (4000000UL)

/**********************************************************************************************************************
 * Global Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Initialise platform timing helpers (timers used by \c platform_timeout_*).
 */
void platform_timing_init(void);

/**
 * \brief Milliseconds remaining until \a target expires.
 *
 * \param target Timeout object previously armed with \c platform_timeout_set.
 * \return Approximate milliseconds left; \c 0 if already expired.
 */
uint32_t platform_timeout_time_left(const platform_timeout_s *target);

#endif /* MIOLINK_PLATFORM_TIMING_H */
