# ============================================================================
# Configure common project parameters that should be set before Pico SDK init
# ============================================================================

cmake_minimum_required(VERSION 3.15)

# Save build type in uppercase for easier comparison
string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

# Use Pico SDK board selection; our board headers in boards/ are searched first (SDK appends its path).
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

# Load all configuration options
include(${CMAKE_CURRENT_LIST_DIR}/cmake/options.cmake)

# Set deoptimized debug mode for debug builds
if(BUILD_TYPE STREQUAL "DEBUG")
    set(PICO_DEOPTIMIZED_DEBUG 1)
else()
    set(PICO_DEOPTIMIZED_DEBUG 0)
endif()

# Run RP2040 at the fastest officially supported system clock (200 MHz).
# Must be set before pico_sdk_init() so hardware_clocks picks the matching PLL setup
# from src/rp2_common/hardware_clocks/include/hardware/clocks.h.
# Note: this option is RP2040-only; on RP2350 the SDK ignores it and uses the default 150 MHz.
set(PICO_USE_FASTEST_SUPPORTED_CLOCK 1)

# ============================================================================
# Import Pico SDK (must run before project() so the toolchain is picked up)
# and define the project
# ============================================================================

# Import Pico SDK
include(${CMAKE_CURRENT_LIST_DIR}/external/pico_sdk.cmake)

# ============================================================================
# Project definition and Pico SDK initialization
# ============================================================================

# Project definition
project(MioLink C CXX ASM)

# Set C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Include configuration
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/config)

# Include boards header
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/boards)

# Import the remaining vendored libraries (FreeRTOS, TinyUSB, Black Magic, SEGGER).
# Must run before pico_sdk_init(): the SDK creates the tinyusb_common_base target
# during pico_sdk_init() and freezes PICO_TINYUSB_PATH and CFG_TUSB_OS at that
# point. external.cmake sets PICO_TINYUSB_PATH to the vendored TinyUSB copy and
# TINYUSB_OPT_OS to OPT_OS_FREERTOS; if it ran later, the SDK would silently
# fall back to its bundled lib/tinyusb and OPT_OS_PICO (non-blocking OSAL).
include(${CMAKE_CURRENT_LIST_DIR}/external/external.cmake)

# Initialize Pico SDK (must be after project)
pico_sdk_init()

# ============================================================================
# Create main executable target and configure common executable parameters
# ============================================================================

# Add executable target
add_executable(MioLink)

# Disable Pico SDK's UART output
pico_enable_stdio_uart(MioLink 0)

# Disable Pico SDK's default printf implementation
pico_set_printf_implementation(MioLink "compiler")

target_compile_definitions(MioLink PRIVATE
    PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS=0
    CONFIG_NUMBER_OF_CORES=${CONFIG_NUMBER_OF_CORES}
    CONFIG_FREERTOS_TICK_RATE_HZ=${CONFIG_FREERTOS_TICK_RATE_HZ}
    CONFIG_GDB_IF_BUFFER_SIZE=${CONFIG_GDB_IF_BUFFER_SIZE}
    CONFIG_SWO_DECODE_BUFFER_SIZE=${CONFIG_SWO_DECODE_BUFFER_SIZE}
)

# ============================================================================
# Generate header files
# ============================================================================

# Generate platform_ident_extra_info.h for compile-time extra info
set(EXTRA_INFO "")
if(PICO_BOARD STREQUAL "auto_rp2040")
    set(EXTRA_INFO "${EXTRA_INFO}, auto-detect")
endif()
if(BUILD_TYPE STREQUAL "DEBUG")
    set(EXTRA_INFO "${EXTRA_INFO}, Debug Build")
endif()

configure_file(
        ${CMAKE_CURRENT_LIST_DIR}/headers_template/platform_ident_extra_info.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/platform_ident_extra_info.h)

# Create version header from git tag
execute_process(COMMAND ${GIT_EXECUTABLE} --git-dir ${CMAKE_CURRENT_LIST_DIR}/../.git describe --tags --always
        OUTPUT_VARIABLE GIT_MIOLINK_REPO_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
configure_file(${CMAKE_CURRENT_LIST_DIR}/headers_template/git_version.h.in ${CMAKE_CURRENT_BINARY_DIR}/git_version.h)

message(STATUS "BlackMagic version: ${GIT_BMP_REPO_VERSION}")
message(STATUS "FreeRTOS-Kernel version: ${GIT_FREERTOS_REPO_VERSION}")
message(STATUS "MioLink version: ${GIT_MIOLINK_REPO_VERSION}")

# Add generated headers to target include directories
target_include_directories(MioLink PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# ============================================================================
# Generate linker scripts
# ============================================================================

# Generate linker scripts
set(PICO_FLASH_SIZE_BYTES_STRING "${PICO_FLASH_SIZE_BYTES}")
if(PICO_PLATFORM MATCHES "^rp2350")
    set(MIOLINK_MEMMAP_RELEASE_TEMPLATE ${CMAKE_CURRENT_LIST_DIR}/linker/memmap_rp2350_release.ld.in)
    set(MIOLINK_MEMMAP_DEBUG_TEMPLATE   ${CMAKE_CURRENT_LIST_DIR}/linker/memmap_rp2350_debug.ld.in)
else()
    set(MIOLINK_MEMMAP_RELEASE_TEMPLATE ${CMAKE_CURRENT_LIST_DIR}/linker/memmap_rp2040_release.ld.in)
    set(MIOLINK_MEMMAP_DEBUG_TEMPLATE   ${CMAKE_CURRENT_LIST_DIR}/linker/memmap_rp2040_debug.ld.in)
endif()
configure_file(${MIOLINK_MEMMAP_RELEASE_TEMPLATE} ${CMAKE_CURRENT_BINARY_DIR}/memmap_release.ld)
configure_file(${MIOLINK_MEMMAP_DEBUG_TEMPLATE}   ${CMAKE_CURRENT_BINARY_DIR}/memmap_debug.ld)

# ============================================================================
# Link other libraries
# ============================================================================

# Add source files
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src)

# Link boards library
target_link_libraries(MioLink PRIVATE
    boards
)

# Link MioLink firmware aggregate
target_link_libraries(MioLink PRIVATE
    miolink_firmware
)

# Link Pico SDK hardware libraries
target_link_libraries(MioLink PRIVATE 
    hardware_pio
    hardware_uart
    hardware_dma
    hardware_timer
    hardware_adc
)

# Link Pico SDK middleware libraries
target_link_libraries(MioLink PRIVATE
    pico_stdlib
    pico_unique_id
    pico_bootrom
)

# Link tinyusb libraries
target_link_libraries(MioLink PRIVATE
    tinyusb_config
    tinyusb_device
    tinyusb_additions
)

# Link FreeRTOS libraries
target_link_libraries(MioLink PRIVATE
    freertos_config
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Link BlackMagic library
target_link_libraries(MioLink PRIVATE
    blackmagic
)

# ============================================================================
# Configure CYW43 support if Wi-Fi is supported by the board
# ============================================================================

if(PICO_CYW43_SUPPORTED)
    target_compile_definitions(MioLink PRIVATE
        PICO_CYW43_SUPPORTED=1
        CYW43_LWIP=0
        CYW43_TASK_PRIORITY=1
        CYW43_TASK_STACK_SIZE=256
    )

    target_link_libraries(MioLink PRIVATE
        pico_cyw43_arch_sys_freertos
    )
endif()

# ============================================================================
# Configure debug parameters
# ============================================================================

if(BUILD_TYPE STREQUAL "DEBUG")
    # Enable debug mode
    target_compile_definitions(MioLink PRIVATE
        ENABLE_DEBUG=1
    )

    # Enable Segger RTT trace if debug is enabled and CONFIG_ENABLE_SEGGER_RTT is ON
    if(CONFIG_ENABLE_SEGGER_RTT)
        target_compile_definitions(MioLink PRIVATE
            ENABLE_SEGGER_RTT=1
        )
        target_link_libraries(MioLink PRIVATE
            segger_rtt
        )
    endif()

    if(CONFIG_ENABLE_SEGGER_SYSVIEW)
        target_compile_definitions(MioLink PRIVATE
            ENABLE_SEGGER_SYSVIEW=1
        )

        target_link_libraries(MioLink PRIVATE
            segger_sysview
        )
    endif()

    # Set debug linker script
    pico_set_linker_script(MioLink ${CMAKE_CURRENT_BINARY_DIR}/memmap_debug.ld)
else()
    # Disable debug mode
    target_compile_definitions(MioLink PRIVATE
        ENABLE_DEBUG=0
    )

    # Set release linker script
    pico_set_linker_script(MioLink ${CMAKE_CURRENT_BINARY_DIR}/memmap_release.ld)
endif()

# ============================================================================
# Configure link options
# ============================================================================

# Print memory usage after linking
target_link_options(MioLink PRIVATE -Xlinker --print-memory-usage)

# Add MioLink output
pico_add_extra_outputs(MioLink)