cmake_minimum_required(VERSION 3.13)

#Set the board 
set(PICO_BOARD pico_w)

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
    # TCLK PIO files (separated into TX and RX)
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclk_tx.pio
    ${CMAKE_CURRENT_LIST_DIR}/pio/tclk_rx.pio
    # Signal detector PIO file
    ${CMAKE_CURRENT_LIST_DIR}/pio/signal_detector.pio
)

# Add include directories
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# Add source files
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/tclk.cpp
)

# Generate PIO headers for TCLK (TX and RX)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclk_tx.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio/tclk_rx.pio)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_cyw43_arch_none
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

# Add signal_monitor executable for hardware debugging
add_executable(signal_monitor
    src/signal_monitor.cpp
)

# Generate PIO header for signal detector
pico_generate_pio_header(signal_monitor ${CMAKE_CURRENT_LIST_DIR}/pio/signal_detector.pio)

# Link libraries for signal_monitor
target_link_libraries(signal_monitor
    pico_stdlib
    pico_cyw43_arch_none  # For LED control on Pico W
    hardware_gpio
    hardware_pio         # For PIO-based signal detection
    hardware_dma         # For DMA with PIO
)

# Enable USB output, disable UART output for signal_monitor
pico_enable_stdio_usb(signal_monitor 1)
pico_enable_stdio_uart(signal_monitor 0)

# Create map/bin/hex/uf2 files for signal_monitor
pico_add_extra_outputs(signal_monitor)

# Add a custom target to flash the Pico
add_custom_target(flash
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.uf2 /media/$ENV{USER}/RPI-RP2
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing ${PROJECT_NAME}.uf2 to Pico"
)

# Add a custom target to flash the signal monitor utility
add_custom_target(flash_monitor
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/signal_monitor.uf2 /media/$ENV{USER}/RPI-RP2
    DEPENDS signal_monitor
    COMMENT "Flashing signal_monitor.uf2 to Pico"
)
