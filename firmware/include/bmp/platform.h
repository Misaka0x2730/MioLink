/*
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

#ifndef PLATFORMS_MIOLINK_PLATFORM_H
#define PLATFORMS_MIOLINK_PLATFORM_H

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "pico/unique_id.h"
#include "timing_rp2040.h"
#include "hardware/pio.h"
#if ENABLE_DEBUG == 1
#include "SEGGER_RTT.h"
#endif
#define PLATFORM_IDENT "(MioLink) "

#define TARGET_SWD_IDLE_CYCLES (8)
#if (TARGET_SWD_IDLE_CYCLES <= 0)
#error "TARGET_SWD_IDLE_CYCLES should be at least 1"
#endif

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp;
#endif

#define PLATFORM_MIOLINK

#define PICO_GPIO_PORT                (0)

#define PLATFORM_HAS_CUSTOM_COMMANDS
#define PLATFORM_HAS_POWER_SWITCH

#define HWVERSION_PIN_0               (14)
#define HWVERSION_PIN_1               (15)

#define TARGET_VOLTAGE_ADC_CHANNEL    (3)
#define TARGET_VOLTAGE_ENABLE_PIN     (3)
#define TARGET_VOLTAGE_FAULT_PIN      (1)

#define NRST_PIN                      (18)

#define LED_ACT_PIN                   (13)
#define LED_ERR_PIN                   (10)
#define LED_SER_PIN                   (12)

#define TARGET_TCK_PIN                (24)
#define TARGET_TDO_PIN                (17)
#define TARGET_TDI_PIN                (28)
#define TARGET_TMS_PIN                (26)
#define TARGET_TMS_DIR_PIN            (27)

#define TDI_PORT                      (PICO_GPIO_PORT)
#define TMS_PORT                      (PICO_GPIO_PORT)
#define TCK_PORT                      (PICO_GPIO_PORT)
#define TDO_PORT                      (PICO_GPIO_PORT)
#define TDI_PIN                       (TARGET_TDI_PIN)
#define TMS_PIN                       (TARGET_TMS_PIN)
#define TCK_PIN                       (TARGET_TCK_PIN)
#define TDO_PIN                       (TARGET_TDO_PIN)

#define SWDIO_PORT                    (PICO_GPIO_PORT)
#define SWCLK_PORT                    (PICO_GPIO_PORT)
#define SWDIO_PIN                     (TARGET_TMS_PIN)
#define SWCLK_PIN                     (TARGET_TCK_PIN)

#define TMS_SET_MODE()                          \
	do {                                        \
		gpio_put(TARGET_TMS_DIR_PIN, 1);        \
		gpio_set_dir(TARGET_TMS_PIN, GPIO_OUT); \
	} while (0)

#define SWDIO_MODE_FLOAT()                        \
	do {                                          \
        gpio_set_dir(TARGET_TMS_PIN, GPIO_IN);    \
        gpio_put(TARGET_TMS_DIR_PIN, 0);          \
	} while (0)

#define SWDIO_MODE_DRIVE()                         \
	do {                                           \
        gpio_put(TARGET_TMS_DIR_PIN, 1);           \
        gpio_set_dir(TARGET_TMS_PIN, GPIO_OUT);    \
	} while (0)

#define gpio_clear(port, pin)           gpio_put(pin, 0)
#define gpio_set(port, pin)             gpio_put(pin, 1)
#define gpio_get(port, pin)             (!!((1ul << pin) & gpio_get_all()))
#define gpio_set_val(port, pin, state)  gpio_put(pin, state)

#define SET_RUN_STATE(state)   running_status = (state)
#define SET_IDLE_STATE(state)  gpio_set_val(PICO_GPIO_PORT, LED_ACT_PIN, state)
#define SET_ERROR_STATE(state) gpio_set_val(PICO_GPIO_PORT, LED_ERR_PIN, state)

#define UART_NUMBER            (1)
#define UART_TX_PIN            (8)
#define UART_RX_PIN            (21)
#define UART_IRQ               (UART1_IRQ)
#define UART_DMA_IRQ           (DMA_IRQ_0)

#define UART_HW                (uart_get_hw(UART_INSTANCE(UART_NUMBER)))


#define UART_PIN_SETUP()                                  \
	do {                                                  \
		gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);   \
    	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);   \
	} while (0)

#define PLATFORM_PRIORITY_LOWEST      (configMAX_PRIORITIES - 5)
#define PLATFORM_PRIORITY_LOW         (configMAX_PRIORITIES - 4)
#define PLATFORM_PRIORITY_NORMAL      (configMAX_PRIORITIES - 3)
#define PLATFORM_PRIORITY_HIGH        (configMAX_PRIORITIES - 2)
#define PLATFORM_PRIORITY_HIGHEST     (configMAX_PRIORITIES - 1)

bool platform_target_is_power_ok(void);

#endif /* PLATFORMS_MIOLINK_PLATFORM_H */