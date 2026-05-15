/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Copyright (C) 2017-2021 Uwe Bonnes <bon@elektron.ikp.physik.tu-darmstadt.de>
 * Copyright (C) 2022-2024 1BitSquared <info@1bitsquared.com>
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Modified by Uwe Bonnes <bon@elektron.ikp.physik.tu-darmstadt.de>
 * Modified by Rachel Mant <git@dragonmux.network>
 * Modified by ALTracer <tolstov_den@mail.ru>
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

#include "pico/unique_id.h"
#include "serialno.h"

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

char serial_no[DFU_SERIAL_LENGTH] = {0}; /**< USB / DFU serial number string filled by \c read_serial_number(). */

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Populate \c serial_no from the RP2040 unique board ID.
 */
void read_serial_number(void)
{
    pico_get_unique_board_id_string(serial_no, sizeof(serial_no));
}
