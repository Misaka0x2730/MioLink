add_library(target_serial INTERFACE)

target_include_directories(target_serial INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB target_serial_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(target_serial INTERFACE
    ${target_serial_sources}
)

target_link_libraries(target_serial INTERFACE
    uart_bridge
)