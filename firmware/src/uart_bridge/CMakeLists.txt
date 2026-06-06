add_library(uart_bridge INTERFACE)

target_include_directories(uart_bridge INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB uart_bridge_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(uart_bridge INTERFACE
    ${uart_bridge_sources}
)
