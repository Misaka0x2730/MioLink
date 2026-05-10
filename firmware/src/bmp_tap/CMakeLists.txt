# ============================================================================
# BlackMagic SWD/JTAG library
# ============================================================================

add_library(bmp_tap INTERFACE)

target_include_directories(bmp_tap INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB bmp_tap_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(bmp_tap INTERFACE
    ${bmp_tap_sources}
)