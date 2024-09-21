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

/* This file provides the platform specific declarations for the native implementation. */

#ifndef PLATFORMS_MIOLINK_PLATFORM_H
#define PLATFORMS_MIOLINK_PLATFORM_H

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "pico/unique_id.h"
#include "timing_rp2040.h"
#include "hardware/pio.h"

#define TARGET_SWD_IDLE_CYCLES (8)
#if (TARGET_SWD_IDLE_CYCLES <= 0)
#error "TARGET_SWD_IDLE_CYCLES should be at least 1"
#endif

#if ENABLE_DEBUG == 1
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp;
#define PLATFORM_IDENT "(MioLink, ENABLE_DEBUG=1) "
#else
#define PLATFORM_IDENT "(MioLink) "
#endif

#define PLATFORM_MIOLINK
#define PLATFORM_MIOLINK_REV_A   (1)
#define PLATFORM_MIOLINK_REV_B   (2)

#define PLATFORM_HAS_TRACESWO
#define PLATFORM_HAS_CUSTOM_COMMANDS
#define PLATFORM_HAS_POWER_SWITCH

#define GDB_ENDPOINT_NOTIF              (0x84)
#define GDB_ENDPOINT                    (0x01)

#define SERIAL_ENDPOINT_NOTIF           (0x85)
#define SERIAL_ENDPOINT                 (0x02)

#ifdef PLATFORM_HAS_TRACESWO
#define TRACESWO_PROTOCOL               (2)
#define TRACE_ENDPOINT                  (0x83)
#endif

#define PICO_GPIO_PORT                  (0)

#define HWVERSION_PIN_0                 (14)
#define HWVERSION_PIN_1                 (15)

#define TARGET_VOLTAGE_ADC_CHANNEL      (3)
#define TARGET_VOLTAGE_ENABLE_PIN       (3)
#define TARGET_VOLTAGE_FAULT_PIN        (1)

#define NRST_PIN                        (18)

#define LED_ACT_PIN                     (13)
#define LED_ERR_PIN                     (10)
#define LED_SER_PIN                     (12)

#define TARGET_TCK_PIN                  (24)
#define TARGET_TDO_PIN                  (17)
#define TARGET_TDI_PIN                  (28)
#define TARGET_TMS_PIN                  (26)
#define TARGET_TMS_DIR_PIN              (27)

#define UART_TX_PIN                     (8)
#define UART_RX_PIN                     (21)

#define gpio_clear(port, pin)           do { sio_hw->gpio_clr = (1ul << pin); } while(0)
#define gpio_set(port, pin)             do { sio_hw->gpio_set = (1ul << pin); } while(0)
#define gpio_get(port, pin)             (!!(gpio_get_all() & (1ul << pin)))
#define gpio_set_val(port, pin, state)  do { if (state) gpio_set(port, pin); else gpio_clear(port, pin); } while(0)

#define SET_RUN_STATE(state)            running_status = (state)
#define SET_IDLE_STATE(state)           gpio_set_val(PICO_GPIO_PORT, LED_ACT_PIN, state)
#define SET_ERROR_STATE(state)          gpio_set_val(PICO_GPIO_PORT, LED_ERR_PIN, state)

#define USB_SERIAL_UART_MAIN            (uart1)
#define USB_SERIAL_UART_TDI_TDO         (uart0)
#define TRACESWO_UART                   (uart0)

#define USB_SERIAL_UART_MAIN_IRQ        (UART_IRQ_NUM(USB_SERIAL_UART_MAIN))
#define USB_SERIAL_UART_TDI_TDO_IRQ     (UART_IRQ_NUM(USB_SERIAL_UART_TDI_TDO))
#define TRACESWO_UART_IRQ               (UART_IRQ_NUM(TRACESWO_UART))

#define USB_SERIAL_TRACESWO_DMA_IRQ     (DMA_IRQ_0)

#define PLATFORM_PRIORITY_LOW           (configMAX_PRIORITIES - 4)
#define PLATFORM_PRIORITY_NORMAL        (configMAX_PRIORITIES - 3)
#define PLATFORM_PRIORITY_HIGH          (configMAX_PRIORITIES - 2)

bool platform_target_is_power_ok(void);

#endif /* PLATFORMS_MIOLINK_PLATFORM_H */