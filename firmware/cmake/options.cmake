# Configuration options for MioLink firmware

# ============================================================================
# Platform Options
# ============================================================================

set(PICO_BOARD "auto_rp2040" CACHE STRING
    "Pico SDK target board
    Options: auto_rp2040, miolink, miolink_pico, pico, pico_w, pico2, pico2_w,
             weact_studio_rp2040_2mb and others supported by the Pico SDK
    Default: auto_rp2040 (auto-detect at runtime on RP2040 boards)")

# ============================================================================
# FreeRTOS Options
# ============================================================================

# Default to single-core for Debug builds, dual-core otherwise
if(BUILD_TYPE STREQUAL "DEBUG")
    set(CONFIG_NUMBER_OF_CORES_DEFAULT "1")
else()
    set(CONFIG_NUMBER_OF_CORES_DEFAULT "2")
endif()

set(CONFIG_NUMBER_OF_CORES "${CONFIG_NUMBER_OF_CORES_DEFAULT}" CACHE STRING
    "Number of MCU cores to use
    Valid values: 1 (single core), 2 (dual core)
    Default: 1 for Debug builds, 2 otherwise"
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
    message(FATAL_ERROR
        "CONFIG_FREERTOS_TICK_RATE_HZ must be between 100 and 10000, got: ${CONFIG_FREERTOS_TICK_RATE_HZ}")
endif()


# ============================================================================
# Debug & Tracing Options
# ============================================================================

option(CONFIG_ENABLE_SEGGER_RTT "Enable Segger RTT trace output in debug builds" ON)
option(CONFIG_ENABLE_SEGGER_SYSVIEW "Enable Segger SystemView trace in single-core debug builds" ON)

# Override the local Segger flags to OFF when their prerequisites are not met, so the configuration
# summary and the consumers in firmware/CMakeLists.txt agree on the actual feature state. Cache
# values are left untouched, so a later reconfigure with valid prerequisites still honours the
# user's original intent.
if(CONFIG_ENABLE_SEGGER_RTT AND NOT BUILD_TYPE STREQUAL "DEBUG")
    message(INFO "CONFIG_ENABLE_SEGGER_RTT=ON requires Debug build; disabling for this build.")
    set(CONFIG_ENABLE_SEGGER_RTT OFF)
endif()
if(CONFIG_ENABLE_SEGGER_SYSVIEW AND NOT BUILD_TYPE STREQUAL "DEBUG")
    message(INFO "CONFIG_ENABLE_SEGGER_SYSVIEW=ON requires Debug build; disabling for this build.")
    set(CONFIG_ENABLE_SEGGER_SYSVIEW OFF)
endif()
if(CONFIG_ENABLE_SEGGER_SYSVIEW AND NOT CONFIG_NUMBER_OF_CORES EQUAL 1)
    message(INFO "CONFIG_ENABLE_SEGGER_SYSVIEW=ON requires CONFIG_NUMBER_OF_CORES=1; disabling for this build.")
    set(CONFIG_ENABLE_SEGGER_SYSVIEW OFF)
endif()

# ============================================================================
# GDB Interface Options
# ============================================================================

set(CONFIG_GDB_IF_BUFFER_SIZE "1024" CACHE STRING
    "Size in bytes of each GDB CDC staging buffer (inbound and outbound)
    Default: 1024"
)

# Validate CONFIG_GDB_IF_BUFFER_SIZE
if(CONFIG_GDB_IF_BUFFER_SIZE LESS 64 OR CONFIG_GDB_IF_BUFFER_SIZE GREATER 65536)
    message(FATAL_ERROR "CONFIG_GDB_IF_BUFFER_SIZE must be between 64 and 65536, got: ${CONFIG_GDB_IF_BUFFER_SIZE}")
endif()

# ============================================================================
# SWO Trace Options
# ============================================================================

set(CONFIG_SWO_DECODE_BUFFER_SIZE "1024" CACHE STRING
    "Size in bytes of the ITM/SWO decoder staging buffer
    Default: 1024"
)

# Validate CONFIG_SWO_DECODE_BUFFER_SIZE
if(CONFIG_SWO_DECODE_BUFFER_SIZE LESS 64 OR CONFIG_SWO_DECODE_BUFFER_SIZE GREATER 65536)
    message(FATAL_ERROR
        "CONFIG_SWO_DECODE_BUFFER_SIZE must be between 64 and 65536, got: ${CONFIG_SWO_DECODE_BUFFER_SIZE}")
endif()

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "=== MioLink Configuration ===")
message(STATUS "  Board: ${PICO_BOARD}")
message(STATUS "  Cores: ${CONFIG_NUMBER_OF_CORES}")
message(STATUS "  FreeRTOS tick: ${CONFIG_FREERTOS_TICK_RATE_HZ} Hz")
message(STATUS "  Segger RTT enabled: ${CONFIG_ENABLE_SEGGER_RTT}")
message(STATUS "  Segger SystemView enabled: ${CONFIG_ENABLE_SEGGER_SYSVIEW}")
message(STATUS "  GDB IF buffer size: ${CONFIG_GDB_IF_BUFFER_SIZE} bytes")
message(STATUS "  SWO decode buffer size: ${CONFIG_SWO_DECODE_BUFFER_SIZE} bytes")
message(STATUS "============================")
