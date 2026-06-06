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

/* Provides main entry point. Initialise subsystems and enter GDB protocol loop. */

/**********************************************************************************************************************
 * Private Includes
 **********************************************************************************************************************/

#include "general.h"

#include "pico/multicore.h"

#include "platform.h"

#include "FreeRTOS.h"
#include "task.h"

#include "uart_bridge.h"
#include "target_serial.h"
#include "usb.h"
#include "usb_cdc.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"

#if defined(ENABLE_RTT)
#include "rtt.h"
#endif

#if defined(PLATFORM_HAS_TRACESWO)
#include "swo.h"
#endif

#if defined(ENABLE_SEGGER_RTT)
#include "SEGGER_RTT.h"
#endif

/**********************************************************************************************************************
 * Private Data
 **********************************************************************************************************************/

static TaskHandle_t gdb_task = NULL; /**< Handle of the GDB worker task. */

/**********************************************************************************************************************
 * Private Functions Prototypes
 **********************************************************************************************************************/

/**
 * \brief Run one iteration of GDB protocol handling: drive target while running, then process one packet.
 */
static void bmp_poll_loop(void);

/**
 * \brief GDB worker task entry — initialises USB/serial/SWO subsystems and loops over the protocol.
 *
 * \param[in] params Unused FreeRTOS task parameter.
 */
_Noreturn static void gdb_thread(void *params);

/**********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/

static void bmp_poll_loop(void)
{
    SET_IDLE_STATE(false);
    while ((gdb_target_running) && (cur_target)) {
        gdb_poll_target();

        /* Check again, as `gdb_poll_target()` may
         * alter these variables. */
        if ((!gdb_target_running) || (!cur_target)) {
            break;
        }
        char c = gdb_if_getchar_to(0);
        if ((c == '\x03') || (c == '\x04')) {
            target_halt_request(cur_target);
        }
#if defined(ENABLE_RTT)
        if (rtt_enabled) {
            poll_rtt(cur_target);
        }
#endif
    }

    SET_IDLE_STATE(true);
    const gdb_packet_s *const packet = gdb_packet_receive();
    /* If port closed and target detached, stay idle */
    if ((packet->data[0] != '\x04') || (cur_target)) {
        SET_IDLE_STATE(false);
    }
    gdb_main(packet);
}

_Noreturn static void gdb_thread(void *params)
{
    (void)params;

    platform_init();

    /* Create the bridge mutex before any task that uses uart_bridge is allowed
     * to run. xSemaphoreCreateMutex must not be called from a critical section. */
    uart_bridge_common_init();

    vTaskSuspendAll();

    usb_cdc_register_listener(USB_CDC_GDB, xTaskGetCurrentTaskHandle(),
        USB_CDC_NOTIF_USB_RX_AVAILABLE | USB_CDC_NOTIF_LINE_STATE_UPDATE);

    blackmagic_usb_init();
    target_serial_init();
#if defined(PLATFORM_HAS_TRACESWO)
    traceswo_task_init();
#endif

    xTaskResumeAll();

    while (1) {
        TRY (EXCEPTION_ALL) {
            bmp_poll_loop();
        }
        CATCH () {
        default:
            gdb_put_packet_error(0xffU);
            target_list_free();
            gdb_outf("Uncaught exception: %s\n", exception_frame.msg);
            morse("TARGET LOST.", true);
        }
    }
}

/**********************************************************************************************************************
 * Public Functions
 **********************************************************************************************************************/

/**
 * \brief Firmware entry point. Brings up debug aids, creates the GDB task, and starts the scheduler.
 */
void main(void)
{
#if defined(ENABLE_SEGGER_RTT)
    SEGGER_RTT_Init();
#endif

#if defined(ENABLE_SEGGER_SYSVIEW)
    traceSTART();
#endif

    multicore_reset_core1();

#if configUSE_CORE_AFFINITY
    const BaseType_t result = xTaskCreateAffinitySet(
        gdb_thread, "target_gdb", GDB_TASK_STACK_SIZE, NULL, GDB_TASK_PRIORITY, GDB_TASK_CORE_AFFINITY, &gdb_task);
#else
    const BaseType_t result =
        xTaskCreate(gdb_thread, "target_gdb", GDB_TASK_STACK_SIZE, NULL, GDB_TASK_PRIORITY, &gdb_task);
#endif

    assert(result == pdPASS);

    vTaskStartScheduler();

    assert(false);
}
