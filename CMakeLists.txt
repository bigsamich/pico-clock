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
    # TCLK PIO file
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclk.pio
)

# Add include directories
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# Add source files
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/tclk.cpp
)

# Add LED and serial test program
add_executable(test_led_serial
    src/test_led_serial.cpp
)

# Generate PIO header for TCLK
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclk.pio)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    pico_multicore
)

# Link libraries for test program
target_link_libraries(test_led_serial
    pico_stdlib
    hardware_gpio
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Enable USB output for test program
pico_enable_stdio_usb(test_led_serial 1)
pico_enable_stdio_uart(test_led_serial 0)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})
pico_add_extra_outputs(test_led_serial)

# Add a custom target to flash the Pico
add_custom_target(flash
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2 /media/$ENV{USER}/RPI-RP2
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing ${PROJECT_NAME}.uf2 to Pico"
)
