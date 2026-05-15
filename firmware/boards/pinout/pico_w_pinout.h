/*
 * This file is part of the MioLink project.
 *
 * Copyright (C) 2026 Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
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

#ifndef MIOLINK_BOARDS_PINOUT_PICO_W_H
#define MIOLINK_BOARDS_PINOUT_PICO_W_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define PICO_W_TARGET_TCK_PIN (10) /**< JTAG/SWD clock pin. */
#define PICO_W_TARGET_TDO_PIN (13) /**< JTAG TDO / SWO input pin. */
#define PICO_W_TARGET_TDI_PIN (12) /**< JTAG TDI pin. */
#define PICO_W_TARGET_TMS_PIN (11) /**< JTAG TMS / SWD SWDIO pin. */

#define PICO_W_TARGET_UART_TX_PIN (8) /**< Target UART TX pin. */
#define PICO_W_TARGET_UART_RX_PIN (9) /**< Target UART RX pin. */

#define PICO_W_TARGET_NRST_PIN (7) /**< Target reset (nRST) GPIO. */

#define CYW43_PIN_WL_DYNAMIC (0) /**< Disables dynamic CYW43 pin remapping. */

#define CYW43_DEFAULT_PIN_WL_REG_ON    (23) /**< CYW43 regulator enable pin. */
#define CYW43_DEFAULT_PIN_WL_DATA_OUT  (24) /**< CYW43 SPI MOSI / data out pin. */
#define CYW43_DEFAULT_PIN_WL_DATA_IN   (24) /**< CYW43 SPI MISO / data in pin (shared with data out). */
#define CYW43_DEFAULT_PIN_WL_HOST_WAKE (24) /**< CYW43 host-wake input pin. */
#define CYW43_DEFAULT_PIN_WL_CLOCK     (29) /**< CYW43 SPI clock pin. */
#define CYW43_DEFAULT_PIN_WL_CS        (25) /**< CYW43 SPI chip-select pin. */
#define CYW43_PIN_WL_HOST_WAKE         (24) /**< Active CYW43 host-wake pin override. */
#define CYW43_PIN_WL_REG_ON            (23) /**< Active CYW43 regulator-enable override. */
#define CYW43_WL_GPIO_COUNT            (3)  /**< Number of CYW43-managed virtual GPIOs. */
#define CYW43_WL_GPIO_LED_PIN          (0)  /**< CYW43 virtual GPIO that drives the on-board LED. */

#endif /* MIOLINK_BOARDS_PINOUT_PICO_W_H */
