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

#ifndef MIOLINK_PLATFORM_CONFIG_THREADS_H
#define MIOLINK_PLATFORM_CONFIG_THREADS_H

/**********************************************************************************************************************
 * Public Definitions
 **********************************************************************************************************************/

#define PLATFORM_PRIORITY_LOW    (tskIDLE_PRIORITY + 1) /**< FreeRTOS priority level: low. */
#define PLATFORM_PRIORITY_NORMAL (tskIDLE_PRIORITY + 2) /**< FreeRTOS priority level: normal. */
#define PLATFORM_PRIORITY_HIGH   (tskIDLE_PRIORITY + 3) /**< FreeRTOS priority level: high. */

#define GDB_TASK_PRIORITY           (PLATFORM_PRIORITY_LOW)    /**< FreeRTOS priority for the GDB worker task. */
#define TRACESWO_TASK_PRIORITY      (PLATFORM_PRIORITY_NORMAL) /**< FreeRTOS priority for the SWO worker task. */
#define TARGET_SERIAL_TASK_PRIORITY (PLATFORM_PRIORITY_NORMAL) /**< FreeRTOS priority for the USB-UART bridge task. */
#define USB_TASK_PRIORITY           (PLATFORM_PRIORITY_HIGH)   /**< FreeRTOS priority for the USB task. */

#define GDB_TASK_CORE_AFFINITY           (0x02) /**< FreeRTOS affinity mask pinning the GDB task to core 1. */
#define TRACESWO_TASK_CORE_AFFINITY      (0x01) /**< FreeRTOS affinity mask pinning the SWO task to core 0. */
#define TARGET_SERIAL_TASK_CORE_AFFINITY (0x01) /**< FreeRTOS affinity mask pinning USB-UART bridge task to core 0. */
#define USB_TASK_CORE_AFFINITY           (0x01) /**< FreeRTOS affinity mask pinning the USB task to core 0. */

#define GDB_TASK_STACK_SIZE           (2936) /**< GDB worker task stack size, in stack words. */
#define TRACESWO_TASK_STACK_SIZE      (512)  /**< SWO worker task stack size, in stack words. */
#define TARGET_SERIAL_TASK_STACK_SIZE (512)  /**< USB-UART bridge task stack size, in stack words. */
#define USB_TASK_STACK_SIZE           (512)  /**< USB task stack size, in stack words. */

#endif /* MIOLINK_PLATFORM_CONFIG_THREADS_H */
