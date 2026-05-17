# ============================================================================
# Pico SDK setting up
#
# Imported before project() so PICO_PLATFORM / PICO_BOARD propagate into the
# toolchain selection. The rest of the vendored dependencies live in
# external.cmake, which is included after pico_sdk_init().
# ============================================================================

# Set Pico SDK path
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/pico-sdk)

# Pico SDK import
include(${CMAKE_CURRENT_LIST_DIR}/pico-sdk/external/pico_sdk_import.cmake)
