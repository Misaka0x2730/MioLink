# Configuration options for MioLink firmware

# ============================================================================
# Platform Options
# ============================================================================

set(PICO_BOARD "auto" CACHE STRING 
    "Pico SDK target board
    Options: auto, miolink, miolink_pico, pico, pico_w, weact_studio_rp2040_2mb
    Default: auto (auto-detect at runtime)")

# ============================================================================
# FreeRTOS Options
# ============================================================================

set(CONFIG_NUMBER_OF_CORES "2" CACHE STRING
    "Number of MCU cores to use
    Valid values: 1 (single core), 2 (dual core)
    Default: 2"
)

# Validate CONFIG_NUMBER_OF_CORES
if(NOT CONFIG_NUMBER_OF_CORES EQUAL 1 AND NOT CONFIG_NUMBER_OF_CORES EQUAL 2)
    message(FATAL_ERROR "CONFIG_NUMBER_OF_CORES must be 1 or 2, got: ${CONFIG_NUMBER_OF_CORES}")
endif()


set(CONFIG_FREERTOS_TICK_RATE_HZ "1000" CACHE STRING
    "FreeRTOS tick rate in Hz
    Default: 1000 (1ms tick)"
)

# Validate CONFIG_FREERTOS_TICK_RATE_HZ
if(CONFIG_FREERTOS_TICK_RATE_HZ LESS 100 OR CONFIG_FREERTOS_TICK_RATE_HZ GREATER 10000)
    message(FATAL_ERROR "CONFIG_FREERTOS_TICK_RATE_HZ must be between 100 and 10000, got: ${CONFIG_FREERTOS_TICK_RATE_HZ}")
endif()


# ============================================================================
# Debug & Tracing Options
# ============================================================================

option(CONFIG_ENABLE_RTT "Enable Segger RTT trace output in debug builds" ON)
option(CONFIG_ENABLE_SYSVIEW "Enable Segger SystemView trace in single-core debug builds" ON)

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "=== MioLink Configuration ===")
message(STATUS "  Board: ${PICO_BOARD}")
message(STATUS "  Cores: ${CONFIG_NUMBER_OF_CORES}")
message(STATUS "  FreeRTOS tick: ${CONFIG_FREERTOS_TICK_RATE_HZ} Hz")
message(STATUS "  RTT enabled: ${CONFIG_ENABLE_RTT}")
message(STATUS "  SystemView enabled: ${CONFIG_ENABLE_SYSVIEW}")
message(STATUS "============================")
