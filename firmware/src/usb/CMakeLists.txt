add_library(usb INTERFACE)

target_include_directories(usb INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB usb_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(usb INTERFACE
    ${usb_sources}
)