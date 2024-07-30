/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Modified 2024 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

#include "general.h"
#include "platform.h"
#include "timing_rp2040.h"

#include "target_internal.h"
#include "gdb_packet.h"

#if defined(PLATFORM_RUBYLINK_REV_C)
#include "multiplexer_task.h"
#endif

extern void trace_tick(void);

#if defined(PLATFORM_MIOLINK)
static int adc_target_voltage_dma_chan = -1;
static uint8_t adc_target_voltage_buf[250] = { 0 };
#endif
static uint16_t target_voltage = 0;

bool cmd_test(target_s *t, int argc, const char **argv)
{
    (void)t;
    (void)argc;
    (void)argv;

    gdb_out("Just for testing\n");

    return true;
}

bool cmd_test1(target_s *t, int argc, const char **argv)
{
    (void)t;
    (void)argc;
    (void)argv;

    gdb_out("Just for testing 1\n");

    return true;
}

const command_s platform_cmd_list[] = {
    {"test", cmd_test, "Just for testing"},
    {"test1", cmd_test1, "Just for testing1"},
    {NULL, NULL, NULL},
};

int platform_hwversion(void)
{
    static int hwversion = -1;

#if defined(PLATFORM_MIOLINK)
    if (hwversion == -1)
    {
        gpio_init(HWVERSION_PIN_0);
        gpio_init(HWVERSION_PIN_1);

        hwversion = gpio_get(PICO_GPIO_PORT, HWVERSION_PIN_1) ? (1 << 1) : 0;
        hwversion |= gpio_get(PICO_GPIO_PORT, HWVERSION_PIN_0) ? (1 << 0) : 0;
    }
#endif

    return hwversion;
}

#if defined(PLATFORM_MIOLINK)
static void __not_in_flash_func(adc_target_voltage_dma_handler)(void)
{
    uint32_t temp = 0;

    for (size_t i = 0; i < sizeof(adc_target_voltage_buf); i++)
    {
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

    dma_channel_acknowledge_irq0(adc_target_voltage_dma_chan);
    dma_channel_set_read_addr(adc_target_voltage_dma_chan, adc_target_voltage_buf, true);
}
#endif

void platform_init(void)
{
    gpio_init(TARGET_TCK_PIN);
    gpio_set_dir(TARGET_TCK_PIN, GPIO_OUT);

    gpio_init(TARGET_TDO_PIN);

    gpio_init(TARGET_TDI_PIN);
    gpio_set_dir(TARGET_TDI_PIN, GPIO_OUT);

    gpio_init(TARGET_TMS_PIN);
    gpio_set_dir(TARGET_TMS_PIN, GPIO_OUT);

    gpio_init(TARGET_TMS_DIR_PIN);
    gpio_set_dir(TARGET_TMS_DIR_PIN, GPIO_OUT);

#if defined(LED_ACT_PIN)
    gpio_init(LED_ACT_PIN);
    gpio_set_dir(LED_ACT_PIN, GPIO_OUT);
#endif

#if defined(LED_ERR_PIN)
    gpio_init(LED_ERR_PIN);
    gpio_set_dir(LED_ERR_PIN, GPIO_OUT);
#endif

#if defined(LED_SER_PIN)
    gpio_init(LED_SER_PIN);
    gpio_set_dir(LED_SER_PIN, GPIO_OUT);
#endif

#if defined(PLATFORM_MIOLINK)
#if defined(NRST_PIN)
    gpio_init(NRST_PIN);
    gpio_set_dir(NRST_PIN, GPIO_OUT);
#endif
#endif

#if defined(PLATFORM_MIOLINK)
#if defined(TARGET_VOLTAGE_ENABLE_PIN)
    gpio_init(TARGET_VOLTAGE_ENABLE_PIN);
    gpio_set_dir(TARGET_VOLTAGE_ENABLE_PIN, GPIO_OUT);
#endif
#endif

#if defined(TARGET_VOLTAGE_FAULT_PIN)
    gpio_init(TARGET_VOLTAGE_FAULT_PIN);
    gpio_set_pulls(TARGET_VOLTAGE_FAULT_PIN, true, false);
#endif

#if defined(PLATFORM_MIOLINK)
    /* Init Vref voltage ADC */
    if (adc_target_voltage_dma_chan == -1)
    {
        adc_init();

        adc_gpio_init(26 + TARGET_VOLTAGE_ADC_CHANNEL);
        adc_select_input(TARGET_VOLTAGE_ADC_CHANNEL);

        adc_fifo_setup(true, true, 1, false, true);

        const uint32_t adc_clock = clock_get_hz(clk_adc);
        adc_set_clkdiv((adc_clock / 1000) - 1); /* Set 1ksps sample rate */

        sleep_ms(1000);

        adc_target_voltage_dma_chan = dma_claim_unused_channel(true);
        dma_channel_config adc_dma_config = dma_channel_get_default_config(adc_target_voltage_dma_chan);

        // Reading from constant address, writing to incrementing byte addresses
        channel_config_set_transfer_data_size(&adc_dma_config, DMA_SIZE_8);
        channel_config_set_read_increment(&adc_dma_config, false);
        channel_config_set_write_increment(&adc_dma_config, true);

        // Pace transfers based on availability of ADC samples
        channel_config_set_dreq(&adc_dma_config, DREQ_ADC);

        dma_channel_configure(adc_target_voltage_dma_chan, &adc_dma_config,
                              adc_target_voltage_buf,    // dst
                              &adc_hw->fifo,  // src
                              sizeof(adc_target_voltage_buf),  // transfer count
                              true            // start immediately
        );

        dma_channel_set_irq0_enabled(adc_target_voltage_dma_chan, true);

        irq_set_exclusive_handler(DMA_IRQ_0, adc_target_voltage_dma_handler);
        irq_set_enabled(DMA_IRQ_0, true);

        adc_run(true);
    }
#endif

    platform_timing_init();
}

void platform_nrst_set_val(bool assert)
{
#if defined(NRST_PIN)
#if defined(PLATFORM_MIOLINK)
    gpio_put(NRST_PIN, assert);
	if (assert)
    {
        platform_delay(10);
	}
#endif
#if defined(PLATFORM_RUBYLINK_REV_C)
    multiplexer_pin_set_value(NRST_PIN, assert);
    platform_delay(10);
#endif
#endif
}

bool platform_nrst_get_val(void)
{
#if defined(NRST_PIN)
#if defined(PLATFORM_MIOLINK)
	return gpio_get (PICO_GPIO_PORT, NRST_PIN);
#endif
#if defined(PLATFORM_RUBYLINK_REV_C)
    return multiplexer_pin_get(NRST_PIN);
#endif
#else
    return false;
#endif
}

#if defined(PLATFORM_HAS_POWER_SWITCH)
bool platform_target_get_power(void)
{
#if defined(PLATFORM_MIOLINK)
    return gpio_get(PICO_GPIO_PORT, TARGET_VOLTAGE_ENABLE_PIN);
#endif

#if defined(PLATFORM_RUBYLINK_REV_C)
    return multiplexer_pin_get(TARGET_VOLTAGE_ENABLE_PIN);
#endif
}

bool platform_target_is_power_ok(void)
{
    return gpio_get(PICO_GPIO_PORT, TARGET_VOLTAGE_FAULT_PIN);
}

bool platform_target_set_power(const bool power)
{
#if defined(PLATFORM_MIOLINK)
    gpio_set_val(PICO_GPIO_PORT, TARGET_VOLTAGE_ENABLE_PIN, power);
    return true;
#endif

#if defined(PLATFORM_RUBYLINK_REV_C)
    multiplexer_pin_set_value(TARGET_VOLTAGE_ENABLE_PIN, power);
    multiplexer_pin_set_value(5, power);
    return true;
#endif
}

uint32_t platform_target_voltage_sense(void)
{
#if defined(PLATFORM_MIOLINK)
    return target_voltage;
#endif

#if defined(PLATFORM_RUBYLINK_REV_C)
    if (platform_target_get_power())
    {
        return 33;
    }
    else
    {
        return 0;
    }
#endif
}

const char *platform_target_voltage(void)
{
    if (platform_target_is_power_ok() == false)
    {
        return "ABSENT!";
    }

    static char ret[] = "0.0V";
    uint32_t val = platform_target_voltage_sense();
    ret[0] = '0' + val / 10U;
    ret[2] = '0' + val % 10U;

    return ret;
}
#else
const char *platform_target_voltage(void)
{
    return NULL;
}
#endif

void platform_target_clk_output_enable(bool enable)
{
	(void)enable;
}

bool platform_spi_init(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_deinit(const spi_bus_e bus)
{
	(void)bus;
	return false;
}

bool platform_spi_chip_select(const uint8_t device_select)
{
	(void)device_select;
	return false;
}

uint8_t platform_spi_xfer(const spi_bus_e bus, const uint8_t value)
{
	(void)bus;
	return value;
}