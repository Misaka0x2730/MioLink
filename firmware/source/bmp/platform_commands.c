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

#include "FreeRTOS.h"
#include "task.h"

#include "general.h"
#include "platform.h"
#include "command.h"
#include "target_internal.h"
#include "gdb_packet.h"

#include "usb_serial.h"
#include "swo.h"

static bool cmd_uart_on_tdi_tdo(target_s *target, int argc, const char **argv);
static bool cmd_rtos_heapinfo(target_s *target, int argc, const char **argv);
static bool cmd_rtos_tasksinfo(target_s *target, int argc, const char **argv);

const command_s platform_cmd_list[] = {
	{"uart_on_tdi_tdo", cmd_uart_on_tdi_tdo, "Use UART pins on TDI and TDO (only in SWD mode): [enable|disable]"},
	{"rtos_heapinfo", cmd_rtos_heapinfo, "Print free FreeRTOS heap size"},
	{"rtos_tasksinfo", cmd_rtos_tasksinfo, "Print info about running tasks"}, {NULL, NULL, NULL}};

bool cmd_uart_on_tdi_tdo(target_s *target, int argc, const char **argv)
{
	(void)target;

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

bool cmd_rtos_heapinfo(target_s *target, int argc, const char **argv)
{
	(void)target;
	(void)argc;
	(void)argv;

	const size_t free_heap = xPortGetFreeHeapSize();
	gdb_outf("Free heap (bytes): %d\n", free_heap);

	return true;
}

#define MACRO_VALUE_STR_WRAP(macro) #macro
#define MACRO_VALUE_STR(macro)      MACRO_VALUE_STR_WRAP(macro)

bool cmd_rtos_tasksinfo(target_s *target, int argc, const char **argv)
{
	(void)target;
	(void)argc;
	(void)argv;

	UBaseType_t tasks_number = uxTaskGetNumberOfTasks();
	TaskStatus_t task_status[10] = {0};

	if (tasks_number > 0) {
		if (uxTaskGetSystemState(task_status, sizeof(task_status) / sizeof(task_status[0]), NULL) == tasks_number) {
			gdb_outf("Total number of tasks: %lu\n", tasks_number);
			gdb_out("Name:                            Min free stack (bytes):\n");
			for (uint32_t i = 0; i < tasks_number; i++) {
				gdb_outf("%-" MACRO_VALUE_STR(configMAX_TASK_NAME_LEN) "s %-5lu\n", task_status[i].pcTaskName,
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