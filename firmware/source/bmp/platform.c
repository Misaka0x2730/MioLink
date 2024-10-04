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
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"

#include "platform.h"
#include "timing_rp2040.h"
#include "target_internal.h"
#include "gdb_packet.h"
#include "command.h"
#include "serialno.h"
#include "version.h"
#include "usb_serial.h"
#include "swo.h"

static int adc_target_voltage_dma_chan = -1;
static uint8_t adc_target_voltage_buf[250] = {0};
static uint16_t target_voltage = 0;
static bool idle_state = false;
static platform_device_type_t device_type = PLATFORM_DEVICE_TYPE_NOT_SET;

char board_ident[BOARD_IDENT_LENGTH] = "";
const char *platform_ident_ptr = "";

bool cmd_uart_on_tdi_tdo(target_s *t, int argc, const char **argv);
bool cmd_rtos_heapinfo(target_s *t, int argc, const char **argv);
bool cmd_rtos_tasksinfo(target_s *t, int argc, const char **argv);

const command_s platform_cmd_list[] = {
	{"uart_on_tdi_tdo", cmd_uart_on_tdi_tdo, "Use UART pins on TDI and TDO (only in SWD mode): [enable|disable]"},
	{"rtos_heapinfo", cmd_rtos_heapinfo, "Print free FreeRTOS heap size"},
	{"rtos_tasksinfo", cmd_rtos_tasksinfo, "Print info about running tasks"}, {NULL, NULL, NULL}};

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

	assert(false);
}

bool cmd_uart_on_tdi_tdo(target_s *t, int argc, const char **argv)
{
	(void)t;

	bool print_status = false;

	if (argc == 1) {
		print_status = true;
	} else if (argc == 2) {
		if (traceswo_uart_is_used(TRACESWO_UART)) {
			print_status = true;
			gdb_out("You should disable TRACESWO before activating UART on TDI and TDO!\n");
		} else if (parse_enable_or_disable(argv[1], &use_uart_on_tdi_tdo))
			print_status = true;
	} else
		gdb_out("Unrecognized command format\n");

	if (print_status) {
		gdb_outf("UART pins on TDI and TDO (only in SWD mode): %s\n", use_uart_on_tdi_tdo ? "enabled" : "disabled");
	}

	return true;
}

bool cmd_rtos_heapinfo(target_s *t, int argc, const char **argv)
{
	const size_t free_heap = xPortGetFreeHeapSize();
	gdb_outf("Free heap (bytes): %d\n", free_heap);

	return true;
}

bool cmd_rtos_tasksinfo(target_s *t, int argc, const char **argv)
{
	UBaseType_t tasks_number = uxTaskGetNumberOfTasks();
	TaskStatus_t task_status[10] = {0};

	if (tasks_number > 0) {
		if (uxTaskGetSystemState(task_status, sizeof(task_status) / sizeof(task_status[0]), NULL) == tasks_number) {
			gdb_outf("Total number of tasks: %lu\n", tasks_number);
			gdb_out("Name:            Min free stack (bytes):\n");
			for (uint32_t i = 0; i < tasks_number; i++) {
				gdb_outf("%-16s %-5lu\n", task_status[i].pcTaskName,
					task_status[i].usStackHighWaterMark * sizeof(StackType_t));
			}
		} else {
			gdb_out("Failed to read tasks info\n");
		}
	} else {
		gdb_out("Incorrect tasks number\n");
	}

	return true;
}

void platform_update_sys_freq(void)
{
	set_sys_clock_hz(configCPU_CLOCK_HZ, true);
}

platform_device_type_t platform_hwtype(void)
{
	return device_type;
}

void platform_update_hwtype(void)
{
	if (device_type == PLATFORM_DEVICE_TYPE_NOT_SET) {
		const int hwversion = platform_hwversion();

		if (hwversion == HWVERSION_PICO) {
			gpio_init(PICO_LED_ACT_PIN);
			gpio_set_dir(PICO_LED_ACT_PIN, GPIO_OUT);
			gpio_put(PICO_LED_ACT_PIN, false);

			adc_init();

			sleep_ms(1000);

			adc_gpio_init(PICO_DETECT_ADC_PIN);
			adc_select_input(PICO_DETECT_ADC_CHANNEL);

			uint16_t adc_result = adc_read();
			/* Just drop the first measurement */
			adc_result = adc_read();

			gpio_init(PICO_LED_ACT_PIN);
			gpio_init(PICO_DETECT_ADC_PIN);

			if (adc_result < PICO_DETECT_ADC_THRESHOLD) {
				cyw43_arch_init();
				device_type = PLATFORM_DEVICE_TYPE_PICO_W;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_PICO;
			}
		} else {
			gpio_init(HWTYPE_PIN_0);
			gpio_set_pulls(HWTYPE_PIN_0, true, false);

			for (uint32_t i = 0; i < 100000; i++)
				__nop();

			if (gpio_get(HWTYPE_PIN_0)) {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK;
			} else {
				device_type = PLATFORM_DEVICE_TYPE_MIOLINK_PICO;
			}
		}
	}
}

int platform_hwversion(void)
{
	static int hwversion = -1;

	if (hwversion == -1) {
		gpio_init(HWVERSION_PIN_0);
		gpio_init(HWVERSION_PIN_1);

		gpio_set_pulls(HWVERSION_PIN_0, true, false);
		gpio_set_pulls(HWVERSION_PIN_1, true, false);

		for (uint32_t i = 0; i < 100000; i++)
			__nop();

		hwversion = gpio_get(HWVERSION_PIN_1) ? (1 << 1) : 0;
		hwversion |= gpio_get(HWVERSION_PIN_0) ? (1 << 0) : 0;

		hwversion++;
	}

	return hwversion;
}

static const platform_target_pins_t miolink_target_pins = {.tck = MIOLINK_TARGET_TCK_PIN,
	.tms = MIOLINK_TARGET_TMS_PIN,
	.tms_dir = MIOLINK_TARGET_TMS_DIR_PIN,
	.tdi = MIOLINK_TARGET_TDI_PIN,
	.tdo = MIOLINK_TARGET_TDO_PIN,
	.uart_tx = MIOLINK_TARGET_UART_TX_PIN,
	.uart_rx = MIOLINK_TARGET_UART_RX_PIN,
	.reset = MIOLINK_TARGET_NRST_PIN,
	.reset_inverted = true};

static const platform_led_pins_t miolink_led_pins = {
	.act = MIOLINK_LED_ACT_PIN, .ser = MIOLINK_LED_SER_PIN, .err = MIOLINK_LED_ERR_PIN};

static const platform_vtref_info_t miolink_adc_info = {.enable_pin = MIOLINK_TARGET_VOLTAGE_ENABLE_PIN,
	.fault_pin = MIOLINK_TARGET_VOLTAGE_FAULT_PIN,
	.adc_channel = MIOLINK_TARGET_VOLTAGE_ADC_CHANNEL};

static const platform_target_pins_t miolink_pico_target_pins = {.tck = MIOLINK_PICO_TARGET_TCK_PIN,
	.tms = MIOLINK_PICO_TARGET_TMS_PIN,
	.tms_dir = MIOLINK_PICO_TARGET_TMS_DIR_PIN,
	.tdi = MIOLINK_PICO_TARGET_TDI_PIN,
	.tdo = MIOLINK_PICO_TARGET_TDO_PIN,
	.uart_tx = MIOLINK_PICO_TARGET_UART_TX_PIN,
	.uart_rx = MIOLINK_PICO_TARGET_UART_RX_PIN,
	.reset = MIOLINK_PICO_TARGET_NRST_PIN,
	.reset_inverted = true};

static const platform_led_pins_t miolink_pico_led_pins = {
	.act = MIOLINK_PICO_LED_ACT_PIN, .ser = MIOLINK_PICO_LED_SER_PIN, .err = MIOLINK_PICO_LED_ERR_PIN};

static const platform_vtref_info_t miolink_pico_adc_info = {.enable_pin = MIOLINK_PICO_TARGET_VOLTAGE_ENABLE_PIN,
	.fault_pin = MIOLINK_PICO_TARGET_VOLTAGE_FAULT_PIN,
	.adc_channel = MIOLINK_PICO_TARGET_VOLTAGE_ADC_CHANNEL};

static const platform_target_pins_t pico_target_pins = {.tck = PICO_TARGET_TCK_PIN,
	.tms = PICO_TARGET_TMS_PIN,
	.tms_dir = PIN_NOT_CONNECTED,
	.tdi = PICO_TARGET_TDI_PIN,
	.tdo = PICO_TARGET_TDO_PIN,
	.uart_tx = PICO_TARGET_UART_TX_PIN,
	.uart_rx = PICO_TARGET_UART_RX_PIN,
	.reset = PICO_TARGET_NRST_PIN,
	.reset_inverted = false};

static const platform_led_pins_t pico_led_pins = {
	.act = PICO_LED_ACT_PIN, .ser = PIN_NOT_CONNECTED, .err = PIN_NOT_CONNECTED};

static const platform_led_pins_t pico_w_led_pins = {
	.act = PIN_NOT_CONNECTED, .ser = PIN_NOT_CONNECTED, .err = PIN_NOT_CONNECTED};

const platform_target_pins_t *platform_get_target_pins(void)
{
	const platform_target_pins_t *p_pins = NULL;

	const platform_device_type_t device_type = platform_hwtype();
	assert(device_type != PLATFORM_DEVICE_TYPE_NOT_SET);

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		p_pins = &miolink_target_pins;
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_target_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_pins = &pico_target_pins;
		break;

	default:
		assert(false);
		break;
	}

	assert(p_pins != NULL);
	return p_pins;
}

const platform_led_pins_t *platform_get_led_pins(void)
{
	const platform_led_pins_t *p_pins = NULL;
	const platform_device_type_t device_type = platform_hwtype();

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		p_pins = &miolink_led_pins;
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_pins = &miolink_pico_led_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
		p_pins = &pico_led_pins;
		break;

	case PLATFORM_DEVICE_TYPE_PICO_W:
		p_pins = &pico_w_led_pins;
		break;

	default:
		break;
	}

	return p_pins;
}

const platform_vtref_info_t *platform_get_adc_info(void)
{
	const platform_vtref_info_t *p_adc_info = NULL;

	const platform_device_type_t device_type = platform_hwtype();

	switch (device_type) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		p_adc_info = &miolink_adc_info;
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		p_adc_info = &miolink_pico_adc_info;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
	case PLATFORM_DEVICE_TYPE_PICO_W:
		break;

	default:
		break;
	}

	return p_adc_info;
}

void platform_set_idle_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	if (p_pins != NULL) {
		if (p_pins->act != PIN_NOT_CONNECTED) {
			gpio_put(p_pins->act, state);
		} else if (platform_hwtype() == PLATFORM_DEVICE_TYPE_PICO_W) {
			if (idle_state != state) {
				cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
			}
		}
	}

	idle_state = state;
}

void platform_toggle_idle_state(void)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	idle_state = !idle_state;

	if (p_pins != NULL) {
		if (p_pins->act != PIN_NOT_CONNECTED) {
			gpio_xor_mask(1ul << p_pins->act);
		} else if (platform_hwtype() == PLATFORM_DEVICE_TYPE_PICO_W) {
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, idle_state);
		}
	}
}

void platform_set_error_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();

	if ((p_pins != NULL) && (p_pins->err != PIN_NOT_CONNECTED)) {
		gpio_put(p_pins->err, state);
	}
}

void platform_set_serial_state(const bool state)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();

	if ((p_pins != NULL) && (p_pins->ser != PIN_NOT_CONNECTED)) {
		gpio_put(p_pins->ser, state);
	}
}

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

void make_board_ident(void)
{
	switch (platform_hwtype()) {
	case PLATFORM_DEVICE_TYPE_MIOLINK:
		snprintf(board_ident, sizeof(board_ident), "Black Magic Probe %s%s", PLATFORM_IDENT_MIOLINK, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK;
		break;

	case PLATFORM_DEVICE_TYPE_MIOLINK_PICO:
		snprintf(
			board_ident, sizeof(board_ident), "Black Magic Probe %s%s", PLATFORM_IDENT_MIOLINK_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_MIOLINK_PICO;
		break;

	case PLATFORM_DEVICE_TYPE_PICO:
		snprintf(board_ident, sizeof(board_ident), "Black Magic Probe %s%s", PLATFORM_IDENT_PICO, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO;
		break;

	case PLATFORM_DEVICE_TYPE_PICO_W:
		snprintf(board_ident, sizeof(board_ident), "Black Magic Probe %s%s", PLATFORM_IDENT_PICO_W, FIRMWARE_VERSION);
		platform_ident_ptr = PLATFORM_IDENT_PICO_W;
		break;

	default:
		assert(false);
		break;
	}
}

const char *platform_ident(void)
{
	return platform_ident_ptr;
}

void platform_init(void)
{
	platform_update_hwtype();
	assert(platform_hwtype() != PLATFORM_DEVICE_TYPE_NOT_SET);

	make_board_ident();

	const platform_led_pins_t *led_pins = platform_get_led_pins();
	assert(led_pins != NULL);

	if (led_pins->act != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->act);
		gpio_set_dir(led_pins->act, GPIO_OUT);
		gpio_put(led_pins->act, false);
	}

	if (led_pins->ser != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->ser);
		gpio_set_dir(led_pins->ser, GPIO_OUT);
		gpio_put(led_pins->ser, false);
	}

	if (led_pins->err != PIN_NOT_CONNECTED) {
		gpio_init(led_pins->err);
		gpio_set_dir(led_pins->err, GPIO_OUT);
		gpio_put(led_pins->err, false);
	}

	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		gpio_init(target_pins->reset);
		gpio_set_dir(target_pins->reset, GPIO_OUT);
		gpio_put(target_pins->reset, !target_pins->reset_inverted);
	}

	const platform_vtref_info_t *p_adc_info = platform_get_adc_info();

	if (p_adc_info != NULL) {
		gpio_init(p_adc_info->enable_pin);
		gpio_set_dir(p_adc_info->enable_pin, GPIO_OUT);
		gpio_put(p_adc_info->enable_pin, false);

		gpio_init(p_adc_info->fault_pin);
		gpio_set_pulls(p_adc_info->fault_pin, true, false);

		/* Init Vref voltage ADC */
		if (adc_target_voltage_dma_chan == -1) {
			adc_init();

			adc_gpio_init(26 + p_adc_info->adc_channel);
			adc_select_input(p_adc_info->adc_channel);

			adc_fifo_setup(true, true, 1, false, true);

			const uint32_t adc_clock = clock_get_hz(clk_adc);
			adc_set_clkdiv(((float)adc_clock / 1000) - 1); /* Set 1ksps sample rate */

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

	platform_timing_init();

	read_serial_number();
}

void platform_nrst_set_val(bool assert)
{
	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		if (assert) {
			gpio_put(target_pins->reset, target_pins->reset_inverted);
			platform_delay(10);
		} else {
			gpio_put(target_pins->reset, !target_pins->reset_inverted);
		}
	}
}

bool platform_nrst_get_val(void)
{
	const platform_target_pins_t *target_pins = platform_get_target_pins();
	assert(target_pins != NULL);

	if (target_pins->reset != PIN_NOT_CONNECTED) {
		return gpio_get(target_pins->reset) == target_pins->reset_inverted;
	} else {
		return false;
	}
}

bool platform_target_get_power(void)
{
	const platform_vtref_info_t *p_adc_info = platform_get_adc_info();

	if ((p_adc_info != NULL) && (p_adc_info->enable_pin != PIN_NOT_CONNECTED)) {
		return gpio_get(p_adc_info->enable_pin);
	} else {
		return false;
	}
}

bool platform_target_is_power_ok(void)
{
	const platform_vtref_info_t *p_adc_info = platform_get_adc_info();

	if ((p_adc_info != NULL) && (p_adc_info->fault_pin != PIN_NOT_CONNECTED)) {
		return gpio_get(p_adc_info->fault_pin);
	} else {
		return true;
	}
}

bool platform_target_set_power(const bool power)
{
	const platform_vtref_info_t *p_adc_info = platform_get_adc_info();

	if ((p_adc_info != NULL) && (p_adc_info->enable_pin != PIN_NOT_CONNECTED)) {
		gpio_put(p_adc_info->enable_pin, power);
	}
	return true;
}

uint32_t platform_target_voltage_sense(void)
{
	return target_voltage;
}

const char *platform_target_voltage(void)
{
	if (platform_get_adc_info() == NULL) {
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

#if ENABLE_SYSVIEW_TRACE
uint32_t SEGGER_SYSVIEW_X_GetTimestamp(void)
{
	return time_us_32();
}
#endif

void tud_dfu_runtime_reboot_to_dfu_cb(void)
{
	const platform_led_pins_t *p_pins = platform_get_led_pins();
	uint32_t pin_mask = 0;

	if (p_pins != NULL) {
		if (p_pins->err == PIN_NOT_CONNECTED) {
			if (p_pins->act != PIN_NOT_CONNECTED) {
				pin_mask |= (1ul << p_pins->act);
			}
		} else {
			pin_mask |= (1ul << p_pins->err);
		}
	}
	/* Error pin is used as boot activity pin, enable all boot modes */
	reset_usb_boot(pin_mask, 0);
}