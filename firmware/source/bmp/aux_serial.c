/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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

#include "general.h"
#include "platform.h"
#include "usb_serial.h"
#include "aux_serial.h"

static uint32_t aux_serial_active_baud_rate;

static char aux_serial_receive_buffer[AUX_UART_BUFFER_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t aux_serial_receive_write_index = 0;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t aux_serial_receive_read_index = 0;

static char aux_serial_transmit_buffer[AUX_UART_BUFFER_SIZE];

/*static int dma_tx_channel = -1;
static int dma_rx_channel = -1;
static dma_channel_config tx_config = { 0 };
static dma_channel_config rx_config = { 0 };*/

static void aux_serial_set_baudrate(const uint32_t baud_rate)
{
	uart_set_baudrate(USBUSART, baud_rate);
	aux_serial_active_baud_rate = baud_rate;
}

void aux_serial_init(void)
{
	/* Setup UART parameters */
	UART_PIN_SETUP();

	uart_init(USBUSART, 38400);
	aux_serial_set_baudrate(38400);

	uart_set_irq_enables(USBUSART, true, false);

	irq_set_exclusive_handler(UART_IRQ, aux_serial_receive_isr);
	irq_set_enabled(USBUSART, true);

	/*dma_tx_channel = dma_claim_unused_channel(true);
	tx_config = dma_channel_get_default_config(dma_tx_channel);	
	channel_config_set_transfer_data_size(tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(tx_config, true);
    channel_config_set_write_increment(tx_config, false);
    channel_config_set_dreq(tx_config, DREQ_UART1_TX);
	
	dma_rx_channel = dma_claim_unused_channel(true);*/
}

void aux_serial_set_encoding(const cdc_line_coding_t *const coding)
{
	uint32_t stop_bits = 2;
	switch (coding->stop_bits)
	{
	case USB_CDC_1_STOP_BITS:
	case USB_CDC_1_5_STOP_BITS:
		stop_bits = 1;
		break;
	case USB_CDC_2_STOP_BITS:
	default:
		break;
	}

	uart_parity_t parity = UART_PARITY_NONE;
	switch (coding->parity)
	{
	case USB_CDC_NO_PARITY:
	default:
		break;
	case USB_CDC_ODD_PARITY:
		parity = UART_PARITY_ODD;
		break;
	case USB_CDC_EVEN_PARITY:
		parity = UART_PARITY_EVEN;
		break;
	}

	uint8_t data_bits = 8;

	if (coding->data_bits <= 8)
	{
		data_bits = coding->data_bits;
	}

	aux_serial_set_baudrate(coding->bit_rate);
	uart_set_format(USBUSART, data_bits, stop_bits, parity);
}

void aux_serial_get_encoding(cdc_line_coding_t *const coding)
{
	coding->bit_rate = aux_serial_active_baud_rate;

	if (UART_HW->lcr_h & UART_UARTLCR_H_STP2_LSB)
	{
		coding->stop_bits = USB_CDC_2_STOP_BITS;
	}
	else
	{
		coding->stop_bits = USB_CDC_1_STOP_BITS;
	}

	if (UART_HW->lcr_h & UART_UARTLCR_H_PEN_LSB)
	{
		if (UART_HW->lcr_h & UART_UARTLCR_H_EPS_LSB)
		{
			coding->parity = USB_CDC_EVEN_PARITY;
		}
		else
		{
			coding->parity = USB_CDC_ODD_PARITY;
		}
	}
	else
	{
		coding->parity = USB_CDC_NO_PARITY;
	}

	coding->data_bits = ((UART_HW->lcr_h >> UART_UARTLCR_H_WLEN_LSB) & UART_UARTLCR_H_WLEN_BITS) + 5;
}

char *aux_serial_current_transmit_buffer(void)
{
	return aux_serial_transmit_buffer;
}

size_t aux_serial_transmit_buffer_fullness(void)
{
	return 0;
}

void aux_serial_send(const size_t len)
{
	uart_write_blocking(UART_INSTANCE, aux_serial_transmit_buffer, len);
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
void USBUART_ISR(void)
{
	bool flush = uart_is_interrupt_source(UART_INSTANCE, UART_INT_RT);

	while (!uart_is_rx_fifo_empty(UART_INSTANCE))
	{
		const char c = uart_recv(UART_INSTANCE);

		/* If the next increment of rx_in would put it at the same point
		* as rx_out, the FIFO is considered full.
		*/
		if (((aux_serial_receive_write_index + 1U) % AUX_UART_BUFFER_SIZE) != aux_serial_receive_read_index)
		{
			/* insert into FIFO */
			aux_serial_receive_buffer[aux_serial_receive_write_index++] = c;

			/* wrap out pointer */
			if (aux_serial_receive_write_index >= AUX_UART_BUFFER_SIZE)
				aux_serial_receive_write_index = 0;
		} else
			flush = true;
	}

	if (flush)
	{
		/* forcibly empty fifo if no USB endpoint */
		if (usb_get_config() != 1)
		{
			aux_serial_receive_read_index = aux_serial_receive_write_index;
			return;
		}

		char packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint8_t buf_out = aux_serial_receive_read_index;

		/* copy from uart FIFO into local usb packet buffer */
		while (aux_serial_receive_write_index != buf_out && packet_size < CDCACM_PACKET_SIZE)
		{
			packet_buf[packet_size++] = aux_serial_receive_buffer[buf_out++];

			/* wrap out pointer */
			if (buf_out >= AUX_UART_BUFFER_SIZE)
				buf_out = 0;
		}

		/* advance fifo out pointer by amount written */
		aux_serial_receive_read_index += usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, packet_buf, packet_size);
		aux_serial_receive_read_index %= AUX_UART_BUFFER_SIZE;
	}
}