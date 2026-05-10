# ============================================================================
# BlackMagic port general functions (ADIv5 SWD, crc, serial, GDB iface)
# ============================================================================

add_library(bmp_general INTERFACE)

target_include_directories(bmp_general INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB bmp_general_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(bmp_general INTERFACE
    ${bmp_general_sources}
)