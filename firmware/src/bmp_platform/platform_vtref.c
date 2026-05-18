/*
 * This file was originally part of Black Magic Debug project.
 *
 * Modified for MioLink project.
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

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "dma_ex.h"

#include "platform.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

#define ADC_TARGET_VOLTAGE_BUF_SIZE    (250)   /**< Number of VTref ADC samples buffered per DMA transfer cycle. */
#define ADC_TARGET_VOLTAGE_SAMPLE_RATE (1000U) /**< VTref ADC sample rate in Hz (1 ksps). */

/**
 * \brief Convert a sum of \ref ADC_TARGET_VOLTAGE_BUF_SIZE 8-bit ADC samples to target voltage in 100 mV units.
 *
 * Reference formula: V(100mV) = (((N / 256) * 3.3) * 2) * 10, where N is the average sample value
 * and the factor of 2 compensates for the on-board VTref voltage divider.
 * Substituting N = sum / ADC_TARGET_VOLTAGE_BUF_SIZE and simplifying yields the integer form:
 * V(100mV) = (sum * 33) / (ADC_TARGET_VOLTAGE_BUF_SIZE * 128).
 *
 * \param[in] sum Sum of \ref ADC_TARGET_VOLTAGE_BUF_SIZE consecutive 8-bit ADC samples.
 * \return Target voltage in 100 mV units (e.g. \c 33 ≈ 3.3 V).
 */
#define ADC_TARGET_VOLTAGE_FROM_SUM(sum) (((sum) * 33U) / (ADC_TARGET_VOLTAGE_BUF_SIZE * 128U))

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

/**
 * \brief DMA channel for ADC-buffer transfers; \ref DMA_EX_CHANNEL_UNCLAIMED until claimed.
 */
static int adc_target_voltage_dma_chan = DMA_EX_CHANNEL_UNCLAIMED;
static uint8_t adc_target_voltage_buf[ADC_TARGET_VOLTAGE_BUF_SIZE] = {0}; /**< Rolling buffer for VTref ADC readings. */
static uint16_t target_voltage = 0; /**< Latest target voltage in units of 100 mV. */

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief DMA IRQ handler for the VTref ADC sampler.
 *
 * Averages \c adc_target_voltage_buf into \c target_voltage and rearms.
 */
static void adc_target_voltage_dma_handler(void);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static void adc_target_voltage_dma_handler(void)
{
    if ((adc_target_voltage_dma_chan == DMA_EX_CHANNEL_UNCLAIMED) ||
        (dma_channel_get_irq1_status((uint)adc_target_voltage_dma_chan) == false)) {
        return;
    }

    uint32_t temp = 0;

    for (size_t i = 0; i < sizeof(adc_target_voltage_buf); i++) {
        temp += adc_target_voltage_buf[i];
    }

    temp = ADC_TARGET_VOLTAGE_FROM_SUM(temp);

    target_voltage = (uint16_t)(temp);

    dma_channel_acknowledge_irq1((uint)adc_target_voltage_dma_chan);
    dma_channel_set_read_addr((uint)adc_target_voltage_dma_chan, &adc_hw->fifo, false);
    dma_channel_set_write_addr((uint)adc_target_voltage_dma_chan, adc_target_voltage_buf, false);
    dma_channel_set_trans_count((uint)adc_target_voltage_dma_chan, sizeof(adc_target_voltage_buf), true);
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

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
        if (adc_target_voltage_dma_chan == DMA_EX_CHANNEL_UNCLAIMED) {
            adc_init();

            adc_gpio_init(ADC_BASE_PIN + p_vtref_info->adc_channel);
            adc_select_input(p_vtref_info->adc_channel);

            adc_fifo_setup(true, true, 1, false, true);

            const uint32_t adc_clock = clock_get_hz(clk_adc);
            adc_set_clkdiv(((float)adc_clock / ADC_TARGET_VOLTAGE_SAMPLE_RATE) - 1);

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

            dma_channel_acknowledge_irq1((uint)adc_target_voltage_dma_chan);
            dma_irqn_set_channel_enabled(1, (uint)adc_target_voltage_dma_chan, true);

            irq_add_shared_handler(
                DMA_IRQ_1, adc_target_voltage_dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
            irq_set_enabled(DMA_IRQ_1, true);

            adc_run(true);
        }
    }
}

/**
 * \brief Whether the target VTref enable line is currently asserted.
 *
 * \return \c true if the enable pin is driven high (target powered by probe); \c false otherwise.
 */
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

/**
 * \brief Enable or disable the target VTref output when the board supports it.
 *
 * \param[in] power \c true to drive VTref, \c false to release it.
 * \return Always \c true.
 */
bool platform_target_set_power(const bool power)
{
    const platform_vtref_info_t *p_vtref_info = platform_get_vtref_info();

    if ((p_vtref_info != NULL) && (p_vtref_info->enable_pin != PIN_NOT_CONNECTED)) {
        gpio_put(p_vtref_info->enable_pin, power);
    }
    return true;
}

/**
 * \brief Latest averaged target voltage in units of 100 mV.
 *
 * \return Voltage in 100 mV units (e.g. \c 33 ≈ 3.3 V).
 */
uint32_t platform_target_voltage_sense(void)
{
    return target_voltage;
}

/**
 * \brief Format the target voltage as a short ASCII string ("3.3V", "ABSENT!", "Not supported").
 *
 * \return Pointer to a statically managed string buffer.
 */
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
