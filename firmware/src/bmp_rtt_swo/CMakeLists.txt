# ============================================================================
# BlackMagic RTT and SWO support
# ============================================================================

add_library(bmp_rtt_swo INTERFACE)

target_include_directories(bmp_rtt_swo INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB bmp_rtt_swo_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(bmp_rtt_swo INTERFACE
    ${bmp_rtt_swo_sources}
)