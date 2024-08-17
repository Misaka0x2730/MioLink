//
// Created by user on 08.08.2024.
//

#ifndef MIOLINK_TAP_PIO_COMMON_H
#define MIOLINK_TAP_PIO_COMMON_H

#include "general.h"
#include "hardware/pio.h"

#define TARGET_SWD_PIO                  (pio0)
#define TARGET_SWD_PIO_SM_SEQ           (0)
#define TARGET_SWD_PIO_SM_ADIV5         (1)

#define TARGET_JTAG_PIO                 (pio0)
#define TARGET_JTAG_PIO_SM_NEXT_CYCLE   (0)
#define TARGET_JTAG_PIO_SM_TMS_SEQ      (1)
#define TARGET_JTAG_PIO_SM_TDI_TDO_SEQ  (2)
#define TARGET_JTAG_PIO_SM_TDI_SEQ      (3)

static inline __attribute__((always_inline)) void tap_pio_common_push_data(PIO pio, uint32_t sm, uint32_t data)
{
    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0);
    pio->txf[sm] = data;
}

static inline __attribute__((always_inline)) uint32_t tap_pio_common_read_data(PIO pio, uint32_t sm)
{
    while((pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))) != 0);
    return pio->rxf[sm];
}

static inline __attribute__((always_inline)) void tap_pio_common_clear_fifos(PIO pio, uint32_t sm)
{
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
}

static inline __attribute__((always_inline)) bool tap_pio_common_rx_is_not_empty(PIO pio, uint32_t sm)
{
    return ((pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))) == 0);
}

static inline __attribute__((always_inline)) bool tap_pio_common_is_not_tx_stalled(PIO pio, uint32_t sm)
{
    pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
    return ((pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0);
}

static inline __attribute__((always_inline)) void tap_pio_common_wait_for_tx_stall(PIO pio, uint32_t sm)
{
    pio->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + sm));
    while((pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm))) == 0);
}

static inline __attribute__((always_inline)) void tap_pio_common_wait_for_rx_level(PIO pio, uint32_t sm, const uint8_t level)
{
    while (((pio->flevel & (PIO_FLEVEL_RX0_BITS << sm*8)) >> (PIO_FLEVEL_RX0_LSB + 8*sm)) < level);
}

static inline __attribute__((always_inline)) void tap_pio_common_wait_until_tx_is_not_full(PIO pio, uint32_t sm)
{
    while((pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + sm))) != 0);
}

static inline __attribute__((always_inline)) void tap_pio_common_enable_sm(PIO pio, uint32_t sm)
{
    pio->ctrl |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

static inline __attribute__((always_inline)) void tap_pio_common_disable_sm(PIO pio, uint32_t sm)
{
    pio->ctrl &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

static inline __attribute__((always_inline)) void tap_pio_common_disable_input_sync(PIO pio, uint32_t pin)
{
    pio->input_sync_bypass |= (1u << pin);
}

#endif //MIOLINK_TAP_PIO_COMMON_H