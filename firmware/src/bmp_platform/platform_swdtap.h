/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2025 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#ifndef MIOLINK_PLATFORM_SWDTAP_H
#define MIOLINK_PLATFORM_SWDTAP_H

/**********************************************************************************************************************
 * Public Includes
 **********************************************************************************************************************/

#include "general.h"

/**********************************************************************************************************************
 * Public Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Output a multi-word SWD bit sequence on the SWD PIO state machine.
 *
 * \param[in] tms_states   Words of line data; total valid bits equals \a clock_cycles (see \c swdptap.c).
 * \param[in] clock_cycles Total number of SWD bits to shift out.
 */
void swdptap_seq_out_buffer(const uint32_t *tms_states, const size_t clock_cycles);

/**
 * \brief ADIv5 write without ACK/payload verification (fast path).
 *
 * \param[in] request Packed ADIv5 request byte.
 * \param[in] data    32-bit payload to write.
 * \return Raw status/ACK nibble from the probe side (see ADIv5).
 */
uint8_t swdtap_adiv5_write_no_check(const uint8_t request, const uint32_t data);

/**
 * \brief ADIv5 read without ACK/payload verification (fast path).
 *
 * \param[in]  request Packed ADIv5 request byte.
 * \param[out] data    Receives 32-bit payload on success.
 * \return Raw status/ACK nibble from the probe side.
 */
uint8_t swdtap_adiv5_read_no_check(const uint8_t request, uint32_t *data);

/**
 * \brief ADIv5 write with ACK and parity checks.
 *
 * \param[in] request Packed ADIv5 request byte.
 * \param[in] data    32-bit payload to write.
 * \return ACK/status; fails if response does not match expected parity/ACK.
 */
uint8_t swdtap_adiv5_write_check(const uint8_t request, const uint32_t data);

/**
 * \brief ADIv5 read with ACK and parity checks.
 *
 * \param[in]  request Packed ADIv5 request byte.
 * \param[out] data    Receives 32-bit payload on success.
 * \param[out] parity  Receives observed parity bit as reported by the check path.
 * \return ACK/status; fails if response does not match expected parity/ACK.
 */
uint8_t swdtap_adiv5_read_check(const uint8_t request, uint32_t *data, bool *parity);

#endif /* MIOLINK_PLATFORM_SWDTAP_H */
