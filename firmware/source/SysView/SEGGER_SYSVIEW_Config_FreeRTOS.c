/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_FreeRTOS.c
Purpose : Sample setup configuration of SystemView with FreeRTOS.
Revision: $Rev: 7745 $
*/
#include "FreeRTOS.h"
#include "task.h"
#include "SEGGER_SYSVIEW.h"

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "MioLink"

// The target device name
#define SYSVIEW_DEVICE_NAME     "RP2040 (Dual Core Cortex-M0+)"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (1000000)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        configCPU_CLOCK_HZ

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x20000000)

/*********************************************************************
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME",D="SYSVIEW_DEVICE_NAME",O=FreeRTOS,C=Dual Cortex-M0+");
  SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick,I#27=DMA_IRQ_0,I#36=UART0_IRQ,I#37=UART1_IRQ,I#19=TIMER_IRQ_3,I#21=RP2040_USB,I#29=IO_IRQ_BANK0");
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ,
                      &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

inline unsigned int SysView_EnterCritical()
{
    if (portCHECK_IF_IN_ISR())
    {
        return portSET_INTERRUPT_MASK_FROM_ISR();
    }
    else
    {
        portENTER_CRITICAL();
        return 0;
    }
}

inline void SysView_ExitCritical(unsigned int isr_mask)
{
    if (portCHECK_IF_IN_ISR())
    {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(isr_mask);
    }
    else
    {
        portEXIT_CRITICAL();
    }
}

/*************************** End of file ****************************/