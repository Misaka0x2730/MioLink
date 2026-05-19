/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "dma_ex.h"

#include "platform.h"

#include "tap_pio.h"

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief DMA channel claimed for PIO TX transfers; \ref DMA_EX_CHANNEL_UNCLAIMED until first use.
 *
 * \note A single channel is shared by both SWD and JTAG TAP transports. This is safe only because
 *       Black Magic drives the TAP exclusively from the GDB task (BMP probes never run SWD and
 *       JTAG concurrently — selecting one transport reconfigures the same \c pio0 block via
 *       \c swdptap_init / \c jtagtap_init, and only one is active at a time). If a future caller
 *       ever needs concurrent TAP DMA — including from a different task or core — this channel
 *       must be split per transport (and per state machine) before any concurrent submission.
 */
static int pio_dma_channel = DMA_EX_CHANNEL_UNCLAIMED;

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

void tap_pio_dma_send_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, const uint32_t data_amount)
{
    assert(buffer_send != NULL);
    assert((data_amount > 0) && (data_amount <= TAP_PIO_DMA_BUF_SIZE));

    check_pio_param(pio);
    check_sm_param(sm);

    dma_ex_claim_channel_if_unclaimed(&pio_dma_channel);

    dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);

    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

    dma_channel_wait_for_finish_blocking(pio_dma_channel);
}

uint32_t tap_pio_dma_send_recv_uint32(PIO pio, uint32_t sm, const uint32_t *buffer_send, uint32_t *buffer_recv,
    const uint32_t data_amount, const uint32_t data_amount_to_read)
{
    assert(buffer_send != NULL);
    assert((data_amount > 0) && (data_amount <= TAP_PIO_DMA_BUF_SIZE));

    check_pio_param(pio);
    check_sm_param(sm);

    dma_ex_claim_channel_if_unclaimed(&pio_dma_channel);

    dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_32);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);

    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

    const uint32_t timeout_start_ms = platform_time_ms();
    uint32_t recv_data_amount = 0;
    while ((dma_channel_is_busy(pio_dma_channel)) || (recv_data_amount < data_amount_to_read)) {
        if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
            const uint32_t read_value = pio_sm_get_blocking(pio, sm);
            if ((buffer_recv != NULL) && (recv_data_amount < data_amount_to_read)) {
                buffer_recv[recv_data_amount] = read_value;
            }
            recv_data_amount++;
        }
        /* Watchdog: a stuck SM or a wrong data_amount_to_read would otherwise loop forever and
         * hang the TAP task. Abort the DMA so the channel is reusable on the next call. */
        if ((platform_time_ms() - timeout_start_ms) >= TAP_PIO_OPERATION_TIMEOUT_MS) {
            dma_channel_abort(pio_dma_channel);
            assert(false);
            break;
        }
    }

    __compiler_memory_barrier();

    return recv_data_amount;
}

void tap_pio_dma_send_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, const uint32_t data_amount)
{
    assert(buffer_send != NULL);
    assert((data_amount > 0) && (data_amount <= TAP_PIO_DMA_BUF_SIZE));

    check_pio_param(pio);
    check_sm_param(sm);

    dma_ex_claim_channel_if_unclaimed(&pio_dma_channel);

    dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);

    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

    dma_channel_wait_for_finish_blocking(pio_dma_channel);
}

uint32_t tap_pio_dma_send_recv_uint8(PIO pio, uint32_t sm, const uint8_t *buffer_send, uint8_t *buffer_recv,
    const uint32_t data_amount, const uint32_t data_amount_to_read)
{
    assert(buffer_send != NULL);
    assert((data_amount > 0) && (data_amount <= TAP_PIO_DMA_BUF_SIZE));

    check_pio_param(pio);
    check_sm_param(sm);

    dma_ex_claim_channel_if_unclaimed(&pio_dma_channel);

    dma_channel_config tx_config = dma_channel_get_default_config(pio_dma_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);

    channel_config_set_dreq(&tx_config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(pio_dma_channel, &tx_config, &(pio->txf[sm]), buffer_send, data_amount, true);

    const uint32_t timeout_start_ms = platform_time_ms();
    uint32_t recv_data_amount = 0;
    while ((dma_channel_is_busy(pio_dma_channel)) || (recv_data_amount < data_amount_to_read)) {
        if (pio_sm_is_rx_fifo_empty(pio, sm) == false) {
            const uint8_t read_value = (uint8_t)((pio_sm_get_blocking(pio, sm) >> 24) & 0xFF);
            if ((buffer_recv != NULL) && (recv_data_amount < data_amount_to_read)) {
                buffer_recv[recv_data_amount] = read_value;
            }
            recv_data_amount++;
        }
        /* Watchdog: a stuck SM or a wrong data_amount_to_read would otherwise loop forever and
         * hang the TAP task. Abort the DMA so the channel is reusable on the next call. */
        if ((platform_time_ms() - timeout_start_ms) >= TAP_PIO_OPERATION_TIMEOUT_MS) {
            dma_channel_abort(pio_dma_channel);
            assert(false);
            break;
        }
    }

    __compiler_memory_barrier();

    return recv_data_amount;
}

uint32_t tap_pio_set_sm_freq(PIO pio, uint32_t sm, uint32_t freq, uint32_t max_interface_freq)
{
    const uint32_t min_freq = (max_interface_freq >> 16); /* Max divider = 65536 */
    const uint32_t max_freq = max_interface_freq;

    if (freq < min_freq) {
        freq = min_freq;
    } else if (freq > max_freq) {
        freq = max_freq;
    }

    uint32_t clkdiv_int = (max_interface_freq / freq);
    uint32_t clkdiv_frac = ((((uint64_t)max_interface_freq) << 8) / freq) & 0xFF;

    if (clkdiv_int >= (((uint32_t)UINT16_MAX) + 1)) {
        clkdiv_int = 0;
        clkdiv_frac = 0;
    }

    pio_sm_set_clkdiv_int_frac(pio, sm, clkdiv_int, clkdiv_frac);
    pio_sm_clkdiv_restart(pio, sm);

    return freq;
}