# ============================================================================
# BlackMagic platform library
# ============================================================================

add_library(bmp_platform INTERFACE)

target_include_directories(bmp_platform INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

file(GLOB bmp_platform_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.c")
target_sources(bmp_platform INTERFACE
    ${bmp_platform_sources}
)