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

#ifndef MIOLINK_BOARDS_PINOUT_MIOLINK_REV_B_H
#define MIOLINK_BOARDS_PINOUT_MIOLINK_REV_B_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define MIOLINK_REVB_TARGET_VOLTAGE_ADC_CHANNEL (3) /**< ADC channel that samples target VTref. */
#define MIOLINK_REVB_TARGET_VOLTAGE_ENABLE_PIN  (3) /**< GPIO enabling target power switch. */
#define MIOLINK_REVB_TARGET_VOLTAGE_FAULT_PIN   (1) /**< GPIO that signals power-switch fault. */

#define MIOLINK_REVB_LED_ACT_PIN (13) /**< Activity LED GPIO. */
#define MIOLINK_REVB_LED_ERR_PIN (10) /**< Error LED GPIO. */
#define MIOLINK_REVB_LED_SER_PIN (12) /**< Serial-activity LED GPIO. */

#define MIOLINK_REVB_TARGET_TCK_PIN     (24) /**< JTAG/SWD clock pin. */
#define MIOLINK_REVB_TARGET_TDO_PIN     (17) /**< JTAG TDO / SWO input pin. */
#define MIOLINK_REVB_TARGET_TDI_PIN     (28) /**< JTAG TDI pin. */
#define MIOLINK_REVB_TARGET_TMS_PIN     (26) /**< JTAG TMS / SWD SWDIO pin. */
#define MIOLINK_REVB_TARGET_TMS_DIR_PIN (25) /**< Direction control for level-shifted TMS/SWDIO. */

#define MIOLINK_REVB_TARGET_UART_TX_PIN (8)  /**< Target UART TX pin. */
#define MIOLINK_REVB_TARGET_UART_RX_PIN (21) /**< Target UART RX pin. */

#define MIOLINK_REVB_TARGET_NRST_PIN (18) /**< Target reset (nRST) GPIO. */

#endif /* MIOLINK_BOARDS_PINOUT_MIOLINK_REV_B_H */
