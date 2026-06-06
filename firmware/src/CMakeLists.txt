# Add all subdirectories
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/uart_bridge)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bmp_platform)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bmp_general)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bmp_rtt_swo)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bmp_tap)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/hw_extras)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/pio_programs)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/target_serial)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/usb)

# Aggregate MioLink firmware: top-level .c files plus subsystem libraries.
add_library(miolink_firmware INTERFACE)

target_include_directories(miolink_firmware INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB miolink_firmware_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(miolink_firmware INTERFACE
    ${miolink_firmware_sources}
)

target_link_libraries(miolink_firmware INTERFACE
    uart_bridge
    bmp_platform
    bmp_general
    bmp_rtt_swo
    bmp_tap
    hw_extras
    pio_programs
    target_serial
    usb
)