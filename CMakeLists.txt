cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK cmake file
include(cmake/pico_sdk_import.cmake)

# Set project name and language
project(pico-clock C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Add PIO files
set(PIO_FILES
    # TCLK PIO files
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclkIN.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclkDecode.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclkEncode.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclkOUT.pio
    # MDAT PIO files
    ${CMAKE_CURRENT_LIST_DIR}/pio/mdatIN.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/mdatDecode.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/mdatEncode.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/mdatOUT.pio
)

# Add include directories
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# Add source files
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/tclk.c
    # src/mdat.c is excluded from build but kept in the project
)

# Generate PIO headers for TCLK
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclkIN.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclkDecode.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclkEncode.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclkOUT.pio)

# Generate PIO headers for MDAT
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/mdatIN.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/mdatDecode.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/mdatEncode.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/mdatOUT.pio)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    pico_multicore
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Add a custom target to flash the Pico
add_custom_target(flash
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2 /media/$ENV{USER}/RPI-RP2
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing ${PROJECT_NAME}.uf2 to Pico"
)
