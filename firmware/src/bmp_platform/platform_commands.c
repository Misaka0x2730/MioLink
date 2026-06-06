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

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

#include "command.h"
#include "target_internal.h"
#include "gdb_packet.h"
#include "swo.h"
#include "target_serial.h"

/**********************************************************************************************************************
 * Private Definitions
 **********************************************************************************************************************/

/** Upper bound on the number of FreeRTOS tasks reportable by \c monitor rtos_tasksinfo. */
#define RTOS_TASKSINFO_MAX_TASKS (16U)

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief \c monitor uart_on_tdi_tdo command handler. Toggles routing target serial through TDI/TDO pins.
 *
 * \param[in,out] target Active target (unused).
 * \param[in]     argc   Argument count from the command line.
 * \param[in]     argv   Argument vector.
 * \return Always \c true (command consumed).
 */
static bool cmd_uart_on_tdi_tdo(target_s *target, int argc, const char **argv);

/**
 * \brief \c monitor rtos_heapinfo command handler. Prints free FreeRTOS heap to the GDB output channel.
 *
 * \param[in,out] target Active target (unused).
 * \param[in]     argc   Argument count (unused).
 * \param[in]     argv   Argument vector (unused).
 * \return Always \c true.
 */
static bool cmd_rtos_heapinfo(target_s *target, int argc, const char **argv);

/**
 * \brief \c monitor rtos_tasksinfo command handler. Prints running FreeRTOS tasks and their stack high-water marks.
 *
 * \param[in,out] target Active target (unused).
 * \param[in]     argc   Argument count (unused).
 * \param[in]     argv   Argument vector (unused).
 * \return Always \c true.
 */
static bool cmd_rtos_tasksinfo(target_s *target, int argc, const char **argv);

/**********************************************************************************************************************
 * Public Data
 **********************************************************************************************************************/

/**
 * \brief Black Magic platform-specific monitor command table; consumed by upstream \c command.c.
 */
const command_s platform_cmd_list[] = {
    {"uart_on_tdi_tdo", cmd_uart_on_tdi_tdo, "Use UART pins on TDI and TDO (only in SWD mode): [enable|disable]"},
    {"rtos_heapinfo", cmd_rtos_heapinfo, "Print free FreeRTOS heap size"},
    {"rtos_tasksinfo", cmd_rtos_tasksinfo, "Print info about running tasks"},
    {NULL, NULL, NULL},
};

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static bool cmd_uart_on_tdi_tdo(target_s *target, int argc, const char **argv)
{
    (void)target;

    bool print_status = false;
    bool uart_on_tdi_tdo = false;

    if (argc == 1) {
        print_status = true;
    } else if (argc == 2) {
        if (swo_current_mode != swo_none) {
            print_status = true;
            gdb_out("You should disable TRACESWO before activating UART on TDI and TDO!\n");
        } else if (parse_enable_or_disable(argv[1], &uart_on_tdi_tdo)) {
            print_status = true;
            target_serial_use_uart_on_tdi_tdo(uart_on_tdi_tdo);
        } else {
            print_status = true;
            gdb_out("Invalid argument; use 'enable' or 'disable'\n");
        }
    } else {
        gdb_out("Unrecognized command format\n");
    }

    if (print_status) {
        gdb_outf("UART pins on TDI and TDO (only in SWD mode): %s\n",
            target_serial_uart_on_tdi_tdo_is_used() ? "enabled" : "disabled");
    }

    return true;
}

static bool cmd_rtos_heapinfo(target_s *target, int argc, const char **argv)
{
    (void)target;
    (void)argc;
    (void)argv;

    const size_t free_heap = xPortGetFreeHeapSize();
    gdb_outf("Free heap (bytes): %zu\n", free_heap);

    return true;
}

static bool cmd_rtos_tasksinfo(target_s *target, int argc, const char **argv)
{
    (void)target;
    (void)argc;
    (void)argv;

    UBaseType_t tasks_number = uxTaskGetNumberOfTasks();
    TaskStatus_t task_status[RTOS_TASKSINFO_MAX_TASKS] = {0};

    if (tasks_number == 0) {
        gdb_out("Incorrect tasks number\n");
    } else if (tasks_number > RTOS_TASKSINFO_MAX_TASKS) {
        gdb_outf("Too many tasks (%lu); bump RTOS_TASKSINFO_MAX_TASKS (=%u) and rebuild\n",
            (unsigned long)tasks_number, (unsigned)RTOS_TASKSINFO_MAX_TASKS);
    } else if (uxTaskGetSystemState(task_status, RTOS_TASKSINFO_MAX_TASKS, NULL) != tasks_number) {
        gdb_out("Failed to read tasks info\n");
    } else {
        gdb_outf("Total number of tasks: %lu\n", (unsigned long)tasks_number);
        gdb_out("Name:                            Min free stack (bytes):\n");
        for (UBaseType_t i = 0; i < tasks_number; i++) {
            gdb_outf("%-" MACRO_VALUE_STR(configMAX_TASK_NAME_LEN) "s %-5zu\n", task_status[i].pcTaskName,
                (size_t)task_status[i].usStackHighWaterMark * sizeof(StackType_t));
        }
    }

    return true;
}
