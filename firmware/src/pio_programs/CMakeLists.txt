add_library(pio_programs INTERFACE)

file(GLOB pio_programs_list CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/*.pio")

pico_generate_pio_header(pio_programs ${pio_programs_list})