/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
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

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"
#include "morse.h"

#include "FreeRTOS.h"
#include "task.h"

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief Morse character descriptor: dot/dash bit pattern (LSB-first) and number of valid bits.
 */
typedef struct {
    uint16_t code; /**< Mark/space bits shifted out by \ref morse_update one per tick (1 = mark, 0 = space). */
    uint8_t bits;  /**< Count of valid bits in \c code, including the trailing inter-element gap. */
} morse_char_s;

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

/**
 * \brief Active status string emitted by the morse blinker, or \c NULL when the blinker is idle.
 *
 * Mutated together with \ref msg_index and \ref morse_repeat under \c portENTER_CRITICAL so writers
 * (\ref morse, GDB task on core 1) and the consumer (\ref morse_update, FreeRTOS timer task on core 0)
 * never observe a torn (msg, index, repeat) triple. Read directly by upstream \c command.c (best-effort
 * pointer load on RP2040; ARM Cortex-M0+ aligned word loads are single-copy atomic).
 */
volatile const char *morse_msg = NULL;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief Bit patterns for 'A'..'Z'. Index with \c (uppercase_char - 'A').
 */
static const morse_char_s morse_char_lut[] = {
    {0x001dU, 8U},  // 'A' .-   0b0000000000011101
    {0x0157U, 12U}, // 'B' -... 0b0000000101010111
    {0x05d7U, 14U}, // 'C' -.-. 0b0000010111010111
    {0x0057U, 10U}, // 'D' -..  0b0000000001010111
    {0x0001U, 4U},  // 'E' .    0b0000000000000001
    {0x0175U, 12U}, // 'F' ..-. 0b0000000101110101
    {0x0177U, 12U}, // 'G' --.  0b0000000101110111
    {0x0055U, 10U}, // 'H' .... 0b0000000001010101
    {0x0005U, 6U},  // 'I' ..   0b0000000000000101
    {0x1dddU, 16U}, // 'J' .--- 0b0001110111011101
    {0x01d7U, 12U}, // 'K' -.-  0b0000000111010111
    {0x015dU, 12U}, // 'L' .-.. 0b0000000101011101
    {0x0077U, 10U}, // 'M' --   0b0000000001110111
    {0x0017U, 8U},  // 'N' -.   0b0000000000010111
    {0x0777U, 14U}, // 'O' ---  0b0000011101110111
    {0x05ddU, 14U}, // 'P' .--. 0b0000010111011101
    {0x1d77U, 16U}, // 'Q' --.- 0b0001110101110111
    {0x005dU, 10U}, // 'R' .-.  0b0000000001011101
    {0x0015U, 8U},  // 'S' ...  0b0000000000010101
    {0x0007U, 6U},  // 'T' -    0b0000000000000111
    {0x0075U, 10U}, // 'U' ..-  0b0000000001110101
    {0x01d5U, 12U}, // 'V' ...- 0b0000000111010101
    {0x01ddU, 12U}, // 'W' .--  0b0000000111011101
    {0x0757U, 14U}, // 'X' -..- 0b0000011101010111
    {0x1dd7U, 16U}, // 'Y' -.-- 0b0001110111010111
    {0x0577U, 14U}, // 'Z' --.. 0b0000010101110111
};

/**
 * \brief Index of the next character in \ref morse_msg to fetch; \c SIZE_MAX means the blinker is idle.
 */
static volatile size_t msg_index = SIZE_MAX;

/**
 * \brief \c true when the active message must restart after its terminator (continuous status pattern).
 */
static volatile bool morse_repeat = false;

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Arm the morse status blinker with a new message, or stop it.
 *
 * Atomic with respect to a concurrent \ref morse_update on the other core: the (\ref morse_msg,
 * \ref msg_index, \ref morse_repeat) triple is updated inside a \c portENTER_CRITICAL section so the
 * consumer never observes a half-updated state.
 *
 * \param[in] msg    Uppercase ASCII status string, transmitted character by character, or \c NULL to stop.
 * \param[in] repeat \c true to loop \p msg indefinitely; \c false to emit it once and become idle.
 */
void morse(const char *const msg, const bool repeat)
{
#if CONFIG_BMDA == 1
    if (msg) {
        DEBUG_WARN("%s\n", msg);
    }
    (void)repeat;
#else
    portENTER_CRITICAL();
    morse_repeat = repeat;
    msg_index = msg ? 0U : SIZE_MAX;
    morse_msg = msg;
    portEXIT_CRITICAL();
#endif
}

/**
 * \brief Produce the next bit of the active morse pattern; called from the SYSTICK timer on core 0.
 *
 * The full body runs under \c portENTER_CRITICAL so a concurrent \ref morse call on core 1 cannot
 * retarget (\ref morse_msg, \ref msg_index, \ref morse_repeat) between the dereference and the index
 * advance. The function-local statics \c code and \c bits are touched only here and therefore inherit
 * the same protection.
 *
 * \return \c true to drive the status LED on for this tick, \c false to drive it off (always \c false while idle).
 */
bool morse_update(void)
{
    static uint16_t code = 0U;
    static uint8_t bits = 0U;

    portENTER_CRITICAL();

    if (msg_index == SIZE_MAX) {
        portEXIT_CRITICAL();
        return false;
    }

    if (!bits) {
        char morse_char = morse_msg[msg_index++];
        if (!morse_char) {
            if (morse_repeat) {
                morse_char = morse_msg[0];
                msg_index = 1U;
            } else {
                msg_index = SIZE_MAX;
                portEXIT_CRITICAL();
                return false;
            }
        }
        if ((morse_char >= 'A') && (morse_char <= 'Z')) {
            const uint8_t morse_char_index = (uint8_t)morse_char - 'A';
            code = morse_char_lut[morse_char_index].code;
            bits = morse_char_lut[morse_char_index].bits;
        } else {
            code = 0U;
            bits = 4U;
        }
    }

    const bool result = ((code & 1U) != 0U);
    code >>= 1U;
    --bits;

    portEXIT_CRITICAL();

    return result;
}
