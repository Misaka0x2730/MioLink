# ============================================================================
# Black Magic Probe: which MCU families are compiled in (see blackmagic
# src/target/meson.build libbmd_target_deps).  When an option is OFF, the
# corresponding C macro is omitted so #ifdef CONFIG_* in upstream code works.
# ============================================================================

set(BMP_TARGET_DEFS "")
set(_BMP_TARGET_OPTION_NAMES "")

macro(_bmp_target_option CACHE_VAR HELP_TEXT CONFIG_MACRO)
    option(${CACHE_VAR} "Black Magic probe target: ${HELP_TEXT}" ON)
    list(APPEND _BMP_TARGET_OPTION_NAMES ${CACHE_VAR})
    if(${CACHE_VAR})
        list(APPEND BMP_TARGET_DEFS "${CONFIG_MACRO}=1")
    endif()
endmacro()

_bmp_target_option(BMP_TARGET_CORTEXAR "Cortex-A/R targets" CONFIG_CORTEXAR)
_bmp_target_option(BMP_TARGET_CORTEXM "Cortex-M targets" CONFIG_CORTEXM)
_bmp_target_option(BMP_TARGET_RISCV "RISC-V targets" CONFIG_RISCV)

option(BMP_TARGET_RISCV_ACCEL "RISC-V remote protocol acceleration (only if RISC-V is enabled)" ON)
list(APPEND _BMP_TARGET_OPTION_NAMES BMP_TARGET_RISCV_ACCEL)
if(BMP_TARGET_RISCV AND BMP_TARGET_RISCV_ACCEL)
    list(APPEND BMP_TARGET_DEFS CONFIG_RISCV_ACCEL=1)
endif()

_bmp_target_option(BMP_TARGET_APOLLO3 "Ambiq Apollo3" CONFIG_APOLLO3)
_bmp_target_option(BMP_TARGET_AT32 "Artery AT32" CONFIG_AT32)
_bmp_target_option(BMP_TARGET_CH32 "WCH CH32" CONFIG_CH32)
_bmp_target_option(BMP_TARGET_CH579 "WCH CH579" CONFIG_CH579)
_bmp_target_option(BMP_TARGET_EFM32 "Silicon Labs EFM32" CONFIG_EFM32)
_bmp_target_option(BMP_TARGET_GD32 "GigaDevice GD32" CONFIG_GD32)
_bmp_target_option(BMP_TARGET_HC32 "HDSC HC32" CONFIG_HC32)
_bmp_target_option(BMP_TARGET_LPC "NXP LPC" CONFIG_LPC)
_bmp_target_option(BMP_TARGET_MM32 "MindMotion MM32" CONFIG_MM32)
_bmp_target_option(BMP_TARGET_NRF "Nordic nRF" CONFIG_NRF)
_bmp_target_option(BMP_TARGET_NXP "NXP i.MX / Kinetis (non-LPC)" CONFIG_NXP)
_bmp_target_option(BMP_TARGET_PUYA "Puya" CONFIG_PUYA)
_bmp_target_option(BMP_TARGET_RA "Renesas RA" CONFIG_RA)
_bmp_target_option(BMP_TARGET_RZ "Renesas RZ" CONFIG_RZ)
_bmp_target_option(BMP_TARGET_RP "Raspberry Pi RP2040/RP2350" CONFIG_RP)
_bmp_target_option(BMP_TARGET_SAM "Microchip SAM" CONFIG_SAM)
_bmp_target_option(BMP_TARGET_STM "STMicroelectronics STM32" CONFIG_STM)
_bmp_target_option(BMP_TARGET_TI "Texas Instruments" CONFIG_TI)
_bmp_target_option(BMP_TARGET_TI_ICEPICK "TI ICEpick" CONFIG_TI_ICEPICK)
_bmp_target_option(BMP_TARGET_XILINX "Xilinx" CONFIG_XILINX)

message(STATUS "=== Black Magic probe targets ===")
foreach(_bmp_opt IN LISTS _BMP_TARGET_OPTION_NAMES)
    if(${_bmp_opt})
        message(STATUS "  ${_bmp_opt}: ON")
    else()
        message(STATUS "  ${_bmp_opt}: OFF")
    endif()
endforeach()
