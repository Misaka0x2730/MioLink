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

/* Provides main entry point. Initialise subsystems and enter GDB protocol loop. */

#include "general.h"

#include "pico/multicore.h"

#include "FreeRTOS.h"
#include "task.h"

#include "platform.h"
#include "usb_serial.h"

#ifdef ENABLE_RTT
#include "rtt.h"
#endif

#ifdef PLATFORM_HAS_TRACESWO
#include "swo.h"
#endif

#if ENABLE_DEBUG
#include "SEGGER_RTT.h"
#endif

#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"
#include "usb.h"

static char BMD_ALIGN_DEF(8) gdb_buffer[GDB_PACKET_BUFFER_SIZE + 1U];

#define GDB_TASK_CORE_AFFINITY (0x02) /* Core 1 only */
#define GDB_TASK_STACK_SIZE    (2048)

TaskHandle_t gdb_task;

char *gdb_packet_buffer()
{
	return gdb_buffer;
}

static void bmp_poll_loop(void)
{
	SET_IDLE_STATE(false);
	while (gdb_target_running && cur_target) {
		gdb_poll_target();

		// Check again, as `gdb_poll_target()` may
		// alter these variables.
		if (!gdb_target_running || !cur_target)
			break;
		const char data = gdb_if_getchar_to(0);
		if (data == '\x03' || data == '\x04')
			target_halt_request(cur_target);
#ifdef ENABLE_RTT
		if (rtt_enabled)
			poll_rtt(cur_target);
#endif
	}

	SET_IDLE_STATE(true);
	size_t size = gdb_getpacket(gdb_buffer, GDB_PACKET_BUFFER_SIZE);
	// If port closed and target detached, stay idle
	if (gdb_buffer[0] != '\x04' || cur_target)
		SET_IDLE_STATE(false);
	gdb_main(gdb_buffer, GDB_PACKET_BUFFER_SIZE, size);
}

_Noreturn static void gdb_thread(void *params)
{
	(void)params;

	while (1) {
		TRY (EXCEPTION_ALL) {
			bmp_poll_loop();
		}
		CATCH () {
		default:
			gdb_putpacketz("EFF");
			target_list_free();
			gdb_outf("Uncaught exception: %s\n", exception_frame.msg);
			morse("TARGET LOST.", true);
		}
	}
}

void main(void)
{
#if ENABLE_DEBUG
	SEGGER_RTT_Init();
#endif

#if ENABLE_SYSVIEW_TRACE
	traceSTART();
#endif

	platform_init();
	blackmagic_usb_init();
	usb_serial_init();
	traceswo_task_init();

	platform_update_sys_freq();

	multicore_reset_core1();

#if configUSE_CORE_AFFINITY
	const BaseType_t result = xTaskCreateAffinitySet(
		gdb_thread, "target_gdb", GDB_TASK_STACK_SIZE, NULL, PLATFORM_PRIORITY_LOW, GDB_TASK_CORE_AFFINITY, &gdb_task);
#else
	const BaseType_t result =
		xTaskCreate(gdb_thread, "target_gdb", GDB_TASK_STACK_SIZE, NULL, PLATFORM_PRIORITY_LOW, &gdb_task);
#endif

	assert(result == pdPASS);

	vTaskStartScheduler();

	assert(false);
}