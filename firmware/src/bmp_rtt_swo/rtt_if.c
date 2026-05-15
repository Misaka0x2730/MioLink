/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
 *
 * MIT License
 *
 * Copyright (c) 2021 Koen De Vleeschauwer
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"
#include "hardware/sync.h"
#include "tusb.h"
#include "usb.h"
#include "usb_cdc.h"
#include "target_serial.h"
#include "gdb_if_ex.h"
#include "rtt.h"
#include "rtt_if.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define RTT_IF_USB_PACKET_SIZE \
    (CFG_TUD_CDC_EP_BUFSIZE)         /**< USB CDC bulk endpoint packet size used for chunked RTT transfers. */
#define RTT_IF_TX_TIMEOUT_MS   (25)  /**< Maximum wait, in ms, for USB CDC to accept a chunk before dropping it. */

/**********************************************************************************************************************
 * Private Types
 **********************************************************************************************************************/

/**
 * \brief RTT down-channel (host→target) receive ring buffer state.
 *
 * \note Single-producer / single-consumer lock-free ring.  The producer
 *       (\c rtt_serial_receive_callback, called from \c target_serial_thread on core 0)
 *       writes only \c head; the consumer (\c rtt_getchar, called from the target
 *       command loop) writes only \c tail.  Cross-core visibility is enforced by
 *       explicit \c __dmb() barriers around the \c head / \c tail updates and the
 *       corresponding buffer-byte access — \c volatile alone does not provide CPU
 *       memory-ordering guarantees.
 */
typedef struct rtt_recv_state {
    char buf[RTT_DOWN_BUF_SIZE]; /**< Circular buffer holding host→target RTT bytes. */
    volatile uint32_t head;      /**< Write cursor for \c buf; only the producer writes it. */
    volatile uint32_t tail;      /**< Read cursor for \c buf; only the consumer writes it. */
} rtt_recv_state_t;

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief RTT down-channel receive state shared between USB RX callback and \ref rtt_getchar.
 */
static rtt_recv_state_t s_rtt_recv = {.buf = {0}, .head = 0, .tail = 0};

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Number of bytes still free in \ref s_rtt_recv (one slot reserved to disambiguate full/empty).
 *
 * \return Free byte count.
 */
inline static uint32_t recv_bytes_free(void);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

inline static uint32_t recv_bytes_free(void)
{
    if (s_rtt_recv.tail <= s_rtt_recv.head) {
        return RTT_DOWN_BUF_SIZE - s_rtt_recv.head + s_rtt_recv.tail - 1U;
    }
    return s_rtt_recv.tail - s_rtt_recv.head - 1U;
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief USB-serial RX callback: drain pending bytes and stash them in the RTT down-channel buffer.
 *
 * Called when USB UART has received new data for the target. Must remain fast (runs from USB context).
 */
void rtt_serial_receive_callback(void)
{
    char usb_buf[RTT_IF_USB_PACKET_SIZE];

    const uint32_t len = target_serial_read((uint8_t *)usb_buf, sizeof(usb_buf));

    /* skip flag: drop packet if not enough free buffer space */
    if ((rtt_flag_skip) && (len > recv_bytes_free())) {
        return;
    }

    /* copy data to s_rtt_recv.buf */
    for (uint32_t i = 0; i < len; i++) {
        uint32_t next_recv_head = (s_rtt_recv.head + 1U) % sizeof(s_rtt_recv.buf);
        if (next_recv_head == s_rtt_recv.tail) {
            break; /* overflow */
        }
        s_rtt_recv.buf[s_rtt_recv.head] = usb_buf[i];
        /* Ensure the byte write is globally visible before publishing the new head. */
        __dmb();
        s_rtt_recv.head = next_recv_head;
    }
}

/**
 * \brief Pop one byte from RTT down-channel buffer.
 *
 * \param[in] channel RTT channel index (only \c 0 is implemented).
 * \return Byte value, or \c -1 when the buffer is empty.
 */
int32_t rtt_getchar(const uint32_t channel)
{
    int retval = -1;
    (void)channel;

    if (s_rtt_recv.head == s_rtt_recv.tail) {
        return -1;
    }
    /* Ensure the head read above happens-before the byte load: without this the
     * load could be reordered earlier and observe a slot the producer has not
     * finished writing. */
    __dmb();
    retval = (uint8_t)s_rtt_recv.buf[s_rtt_recv.tail];
    /* Ensure the byte load completes before the tail update is published, so the
     * producer cannot see the freed slot and overwrite it before we are done. */
    __dmb();
    s_rtt_recv.tail = (s_rtt_recv.tail + 1U) % sizeof(s_rtt_recv.buf);

    return retval;
}

/**
 * \brief Whether any bytes are currently buffered for the target on this RTT channel.
 *
 * \param[in] channel RTT channel index (only \c 0 is implemented).
 * \return \c true if no data is available.
 */
bool rtt_nodata(const uint32_t channel)
{
    /* only support reading from down channel 0 */
    if (channel != 0U) {
        return true;
    }

    return s_rtt_recv.head == s_rtt_recv.tail;
}

/**
 * \brief Forward RTT target→host bytes over the USB serial CDC, in \ref RTT_IF_USB_PACKET_SIZE chunks.
 *
 * \param[in] channel RTT channel index (only \c 0 is implemented).
 * \param[in] buf     Source buffer.
 * \param[in] len     Number of bytes to send.
 * \return \a len on success / acceptance; \c 0 if dropped silently because USB stayed full
 *         for \ref RTT_IF_TX_TIMEOUT_MS milliseconds.
 */
uint32_t rtt_write(const uint32_t channel, const char *buf, uint32_t len)
{
    /* only support writing to up channel 0 */
    if (channel != 0U) {
        return len;
    }

    if ((len != 0) && (usb_get_config() == USB_CONFIG_STATE_CONFIGURED) &&
        (gdb_serial_get_dtr() == USB_CDC_DTR_ASSERTED) && (target_serial_get_dtr() == USB_CDC_DTR_ASSERTED)) {
        for (uint32_t pos = 0; pos < len; pos += RTT_IF_USB_PACKET_SIZE) {
            uint32_t plen = MIN(RTT_IF_USB_PACKET_SIZE, len - pos);
            uint32_t start_ms = platform_time_ms();
            while (target_serial_send_to_usb((uint8_t *)(buf + pos), plen, false, false) == false) {
                if (platform_time_ms() - start_ms >= RTT_IF_TX_TIMEOUT_MS) {
                    return 0; /* drop silently */
                }
            }
        }
    }
    return len;
}
