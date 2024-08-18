#include "general.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usb_serial.h"
#include "platform.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"
#include "command.h"
#ifdef ENABLE_RTT
#include "rtt.h"
#endif

#ifdef PLATFORM_HAS_TRACESWO
#include "traceswo.h"
#endif

#include "pico/multicore.h"

#include "usb.h"

#if ENABLE_DEBUG
#include "SEGGER_RTT.h"
#endif

static char BMD_ALIGN_DEF(8) pbuf[GDB_PACKET_BUFFER_SIZE + 1U];

#define GDB_TASK_CORE_AFFINITY     (0x01) /* Core 0 only */

TaskHandle_t gdb_task;

char *gdb_packet_buffer()
{
    return pbuf;
}

static void bmp_poll_loop(void)
{
	SET_IDLE_STATE(false);
	while (gdb_target_running && cur_target)
    {
		gdb_poll_target();

		// Check again, as `gdb_poll_target()` may
		// alter these variables.
		if (!gdb_target_running || !cur_target)
			break;
		char c = gdb_if_getchar_to(0);
		if (c == '\x03' || c == '\x04')
			target_halt_request(cur_target);
        /* TODO: fix this error */
        //platform_pace_poll();
#ifdef ENABLE_RTT
		if (rtt_enabled)
			poll_rtt(cur_target);
#endif
	}

	SET_IDLE_STATE(true);
	size_t size = gdb_getpacket(pbuf, GDB_PACKET_BUFFER_SIZE);
	// If port closed and target detached, stay idle
	if (pbuf[0] != '\x04' || cur_target)
		SET_IDLE_STATE(false);
	gdb_main(pbuf, GDB_PACKET_BUFFER_SIZE, size);
}

_Noreturn static void gdb_thread(void* params)
{
    while (1)
    {
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
    traceSTART();
#endif
    platform_init();
    platform_timing_init();
    blackmagic_usb_init();
    usb_serial_init();
    traceswo_task_init();

    multicore_reset_core1();

    BaseType_t status = xTaskCreate(gdb_thread,
                                    "target_gdb",
									2048,
                                    NULL,
                                    PLATFORM_PRIORITY_NORMAL,
                                    &gdb_task);

#if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(main_task, GDB_TASK_CORE_AFFINITY);
#endif
    vTaskStartScheduler();

	panic("Should not reach here!");
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    panic("Stack overflow!");
}