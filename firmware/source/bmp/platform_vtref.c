/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

#include "general.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "platform.h"

static int adc_target_voltage_dma_chan = -1;
static uint8_t adc_target_voltage_buf[250] = {0};
static uint16_t target_voltage = 0;

static void adc_target_voltage_dma_handler(void)
{
	uint32_t temp = 0;

	for (size_t i = 0; i < sizeof(adc_target_voltage_buf); i++) {
		temp += adc_target_voltage_buf[i];
	}

	/* There is a divisor by 2, so target voltage (hundred of mV) formula is:
     * V(100mv) = (((N/256)*3.3) * 2) * 10, where N - average value of measurements
     * By simplification (since 250 measurements are stored in the buffer),
     * we obtain the following formula:
     * V(100mv) = ((sum * 33) / 32000), where is sum - sum of 250 measurements  */
	temp *= 33;
	temp /= 32000;

	target_voltage = (uint16_t)(temp);

	dma_channel_acknowledge_irq1(adc_target_voltage_dma_chan);
	dma_channel_set_read_addr(adc_target_voltage_dma_chan, &adc_hw->fifo, false);
	dma_channel_set_write_addr(adc_target_voltage_dma_chan, adc_target_voltage_buf, false);
	dma_channel_set_trans_count(adc_target_voltage_dma_chan, sizeof(adc_target_voltage_buf), true);
}

void platform_vtref_init(void)
{
	const platform_vtref_info_t *p_vtref_info = platform_get_vtref_info();

	if (p_vtref_info != NULL) {
		gpio_init(p_vtref_info->enable_pin);
		gpio_set_dir(p_vtref_info->enable_pin, GPIO_OUT);
		gpio_put(p_vtref_info->enable_pin, false);

		gpio_init(p_vtref_info->fault_pin);
		gpio_set_pulls(p_vtref_info->fault_pin, true, false);

		/* Init Vref voltage ADC */
		if (adc_target_voltage_dma_chan == -1) {
			adc_init();

			adc_gpio_init(26 + p_vtref_info->adc_channel);
			adc_select_input(p_vtref_info->adc_channel);

			adc_fifo_setup(true, true, 1, false, true);

			const uint32_t adc_clock = clock_get_hz(clk_adc);
			adc_set_clkdiv(((float)adc_clock / 1000) - 1); /* Set 1ksps sample rate */

			adc_target_voltage_dma_chan = dma_claim_unused_channel(true);
			dma_channel_config adc_dma_config = dma_channel_get_default_config(adc_target_voltage_dma_chan);

			// Reading from constant address, writing to incrementing byte addresses
			channel_config_set_transfer_data_size(&adc_dma_config, DMA_SIZE_8);
			channel_config_set_read_increment(&adc_dma_config, false);
			channel_config_set_write_increment(&adc_dma_config, true);

			// Pace transfers based on availability of ADC samples
			channel_config_set_dreq(&adc_dma_config, DREQ_ADC);

			dma_channel_configure(adc_target_voltage_dma_chan, &adc_dma_config,
				adc_target_voltage_buf,         // dst
				&adc_hw->fifo,                  // src
				sizeof(adc_target_voltage_buf), // transfer count
				true                            // start immediately
			);

			dma_channel_acknowledge_irq0(adc_target_voltage_dma_chan);
			dma_channel_set_irq1_enabled(adc_target_voltage_dma_chan, true);

			irq_set_exclusive_handler(DMA_IRQ_1, adc_target_voltage_dma_handler);
			irq_set_enabled(DMA_IRQ_1, true);

			adc_run(true);
		}
	}
}

bool platform_target_get_power(void)
{
	const platform_vtref_info_t *p_vtref_info = platform_get_vtref_info();

	if ((p_vtref_info != NULL) && (p_vtref_info->enable_pin != PIN_NOT_CONNECTED)) {
		return gpio_get(p_vtref_info->enable_pin);
	}

	return false;
}

bool platform_target_is_power_ok(void)
{
	const platform_vtref_info_t *p_vtref_info = platform_get_vtref_info();

	if ((p_vtref_info != NULL) && (p_vtref_info->fault_pin != PIN_NOT_CONNECTED)) {
		return gpio_get(p_vtref_info->fault_pin);
	}

	return true;
}

bool platform_target_set_power(const bool power)
{
	const platform_vtref_info_t *p_vtref_info = platform_get_vtref_info();

	if ((p_vtref_info != NULL) && (p_vtref_info->enable_pin != PIN_NOT_CONNECTED)) {
		gpio_put(p_vtref_info->enable_pin, power);
	}
	return true;
}

uint32_t platform_target_voltage_sense(void)
{
	return target_voltage;
}

const char *platform_target_voltage(void)
{
	if (platform_get_vtref_info() == NULL) {
		return "Not supported";
	}

	if (platform_target_is_power_ok() == false) {
		return "ABSENT!";
	}

	static char ret[] = "0.0V";
	uint32_t val = platform_target_voltage_sense();
	ret[0] = '0' + val / 10U;
	ret[2] = '0' + val % 10U;

	return ret;
}