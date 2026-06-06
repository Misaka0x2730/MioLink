/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.     See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.     If not, see <http://www.gnu.org/licenses/>.
 */

/* Print decoded swo stream on the usb serial */

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "swo.h"
#include "usb.h"
#include "target_serial.h"
#include "gdb_if_ex.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#if !defined(CONFIG_SWO_DECODE_BUFFER_SIZE)
#error "CONFIG_SWO_DECODE_BUFFER_SIZE must be defined by the build system"
#endif

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief State of the ITM/SWO decoder shared across \c traceswo_decode invocations.
 *
 * The decoder may be entered with a packet split across two UART RX buffers, so this state is preserved between
 * calls until either the current packet completes or \c flush is requested.
 */
typedef struct {
    uint8_t buf[CONFIG_SWO_DECODE_BUFFER_SIZE]; /**< Buffer of decoded ITM payload bytes pending USB forwarding. */
    int buf_len;                                /**< Number of valid bytes currently in \c buf. */
    uint32_t decode_mask;                       /**< Bitmask of ITM stimulus ports being forwarded. */
    int pkt_len;                                /**< Remaining payload bytes in the current SWO packet. */
    bool print;                                 /**< Whether the packet's channel is enabled in \c decode_mask. */
} swo_decoder_s;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/* state is static in case swo packet is astride two buffers */
static swo_decoder_s swo_decoder = {.decode_mask = 1U}; /**< Persistent ITM/SWO decoder state. */

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

bool traceswo_decode(const void *buf, uint16_t len, const bool flush, const bool drop_if_no_space)
{
    const uint8_t *const data = (const uint8_t *)buf;

    if (target_serial_get_available() < sizeof(swo_decoder.buf)) {
        return drop_if_no_space;
    }

    for (uint16_t i = 0; i < len; i++) {
        const uint8_t ch = data[i];
        if (swo_decoder.pkt_len == 0) {                  /* header */
            const uint32_t channel = (uint32_t)ch >> 3U; /* channel number */
            const uint32_t size = ch & 0x7U;             /* drop channel number */
            if (size == 0x01U) {
                swo_decoder.pkt_len = 1; /* SWO packet 0x01XX */
            } else if (size == 0x02U) {
                swo_decoder.pkt_len = 2; /* SWO packet 0x02XXXX */
            } else if (size == 0x03U) {
                swo_decoder.pkt_len = 4; /* SWO packet 0x03XXXXXXXX */
            }
            swo_decoder.print = (swo_decoder.pkt_len != 0) && ((swo_decoder.decode_mask & (1UL << channel)) != 0UL);
        } else if (swo_decoder.pkt_len <= 4) { /* data */
            if (swo_decoder.print) {
                swo_decoder.buf[swo_decoder.buf_len++] = ch;

                if (swo_decoder.buf_len == sizeof(swo_decoder.buf)) {
                    target_serial_send_to_usb(swo_decoder.buf, swo_decoder.buf_len, false, true);
                    swo_decoder.buf_len = 0;
                }
            }

            --swo_decoder.pkt_len;
        } else { /* recover */
            swo_decoder.buf_len = 0;
            swo_decoder.pkt_len = 0;
        }
    }

    if (flush) {
        if ((usb_get_config() == USB_CONFIG_STATE_CONFIGURED) && (target_serial_get_dtr() == USB_CDC_DTR_ASSERTED)) {
            target_serial_send_to_usb(swo_decoder.buf, swo_decoder.buf_len, flush, true);
        }
        swo_decoder.buf_len = 0;
    }

    return true;
}

void traceswo_setmask(uint32_t mask)
{
    swo_decoder.decode_mask = mask;
}
