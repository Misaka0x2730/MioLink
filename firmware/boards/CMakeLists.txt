add_library(boards INTERFACE)

target_include_directories(boards INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/pinout
)