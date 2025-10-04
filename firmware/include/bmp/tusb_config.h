/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Modified by Dmitry Rezvanov <dmitry.rezvanov@yandex.ru>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef MIOLINK_TUSB_CONFIG_H
#define MIOLINK_TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by board.mk
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

// RHPort number used for device can be defined by board.mk, default to port 0
#ifndef BOARD_DEVICE_RHPORT_NUM
#define BOARD_DEVICE_RHPORT_NUM 0
#endif

// RHPort max operational speed can be defined by board.mk
// Default to Highspeed for MCU with internal HighSpeed PHY (can be port specific), otherwise FullSpeed
#ifndef BOARD_DEVICE_RHPORT_SPEED
#if (CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_LPC43XX || CFG_TUSB_MCU == OPT_MCU_MIMXRT10XX || \
	CFG_TUSB_MCU == OPT_MCU_NUC505 || CFG_TUSB_MCU == OPT_MCU_CXD56 || CFG_TUSB_MCU == OPT_MCU_SAMX7X)
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_HIGH_SPEED
#else
#define BOARD_DEVICE_RHPORT_SPEED OPT_MODE_FULL_SPEED
#endif
#endif

// Device mode with rhport and speed defined by board.mk
#if BOARD_DEVICE_RHPORT_NUM == 0
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#elif BOARD_DEVICE_RHPORT_NUM == 1
#define CFG_TUSB_RHPORT1_MODE (OPT_MODE_DEVICE | BOARD_DEVICE_RHPORT_SPEED)
#else
#error "Incorrect RHPort configuration"
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC         2
#define CFG_TUD_MSC         0
#define CFG_TUD_HID         0
#define CFG_TUD_MIDI        0
#define CFG_TUD_VENDOR      1
#define CFG_TUD_DFU_RUNTIME 1

// CDC FIFO size of TX and RX
#if !defined(CFG_TUD_CDC_RX_BUFSIZE)
#define CFG_TUD_CDC_RX_BUFSIZE 2048
#endif

#if !defined(CFG_TUD_CDC_TX_BUFSIZE)
#define CFG_TUD_CDC_TX_BUFSIZE 4096
#endif

#if !defined(CFG_TUD_CDC_EP_BUFSIZE)
#define CFG_TUD_CDC_EP_BUFSIZE 2048
#endif

#if !defined(CFG_TUD_VENDOR_EPSIZE)
#define CFG_TUD_VENDOR_EPSIZE 64
#endif

#if !defined(CFG_TUD_VENDOR_RX_BUFSIZE)
#define CFG_TUD_VENDOR_RX_BUFSIZE 64
#endif

#if !defined(CFG_TUD_VENDOR_TX_BUFSIZE)
#define CFG_TUD_VENDOR_TX_BUFSIZE 2048
#endif

#ifdef __cplusplus
}
#endif

#endif /* MIOLINK_TUSB_CONFIG_H */