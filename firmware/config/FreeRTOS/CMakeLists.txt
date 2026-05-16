add_library(freertos_config INTERFACE)

target_include_directories(freertos_config INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(freertos_config INTERFACE
    hardware_regs
)