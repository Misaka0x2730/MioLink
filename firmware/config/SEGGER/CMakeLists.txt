# SEGGER RTT configuration library
add_library(segger_rtt_config INTERFACE)

target_include_directories(segger_rtt_config INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

# SEGGER SystemView configuration library
add_library(segger_sysview_config INTERFACE)

target_include_directories(segger_sysview_config INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB segger_sysview_config_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(segger_sysview_config INTERFACE
    ${segger_sysview_config_sources}
)