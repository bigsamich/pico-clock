/**
 * @file main.cpp
 * @brief Main application for the Raspberry Pi Pico TCLK protocol implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "tclk.h"

// Configuration
#define TCLK_INPUT_PIN 12    // GPIO pin for TCLK input
#define TCLK_OUTPUT_PIN 13   // GPIO pin for TCLK output
#define BUFFER_SIZE 256      // Size of the buffer for data bytes
#define JSON_BUFFER_SIZE 128 // Size of the JSON buffer

// Function prototypes
void core1_entry();
void process_received_tclk_data(uint8_t* buffer, size_t byte_count, uint64_t timestamp);
void output_tclk_event(uint8_t event_byte, uint64_t timestamp);
void process_serial_command();
void send_timeline(const uint8_t* events, const uint64_t* offsets, size_t count);

// Global variables
static tclk_t tclk;
static volatile bool running = true;

int main() {
    // Initialize stdio
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }
  
    // 5 second loop with a starting message every second
    for (int i = 1; i <= 5; ++i) {
        printf("Starting in %d...\n", 6 - i);
        //Flash the LED to indicate startup for 1 second off and on at 20Hz
        for (int j = 0; j < 20; ++j) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(25);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(25);
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    }
    
    printf("Raspberry Pi Pico Clock Protocol Implementation\n");
    printf("--------------------------------------------\n");
    
    // Initialize the TCLK interface
    if (!tclk_init(&tclk, pio0, TCLK_INPUT_PIN, TCLK_OUTPUT_PIN)) {
        printf("Failed to initialize TCLK interface\n");
        return 1;
    }
    
    printf("TCLK interface initialized (IN: pin %d, OUT: pin %d)\n", 
           TCLK_INPUT_PIN, TCLK_OUTPUT_PIN);
    
    // Start the second core for data processing
    multicore_launch_core1(core1_entry);
    
    // Start the TCLK interface
    if (!tclk_start(&tclk)) {
        printf("Failed to start TCLK interface\n");
        running = false;
        return 1;
    }
    
    printf("TCLK interface started\n");
    
    // Synchronize the TCLK interface (will wait indefinitely until synchronized)
    printf("Starting TCLK synchronization. This will wait until a valid signal is detected...\n");
    if (!tclk_synchronize(&tclk)) {
        printf("Error: Failed to synchronize TCLK interface (invalid tclk structure)\n");
        running = false;
        return 1;
    }
    
    printf("Ready to receive commands. Type 'help' for available commands.\n");
    
    // Main loop - listen for serial commands
    while (running) {
        process_serial_command();
        sleep_ms(10); // Small delay to prevent tight loop
    }
    
    // Clean up
    tclk_stop(&tclk);
    tclk_deinit(&tclk);
    
    printf("TCLK interface stopped\n");
    return 0;
}

// Core 1 entry point - handles data processing
void core1_entry() {
    printf("Data processing started on core 1\n");
    
    uint8_t rx_buffer[BUFFER_SIZE]; // Buffer for received bytes
    int loop_count = 0;
    int empty_count = 0;
    
    while (running) {
        // Periodically toggle the LED to show core1 is running
        if (loop_count % 500 == 0) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(5);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        }
        
        // Debug: Check raw PIO FIFO status
        if (loop_count % 100 == 0) {
            bool fifo_empty = pio_sm_is_rx_fifo_empty(tclk.pio, tclk.sm_rx);
            int fifo_level = pio_sm_get_rx_fifo_level(tclk.pio, tclk.sm_rx);
            printf("DEBUG: PIO RX FIFO status - %s, level: %d\n",
                   fifo_empty ? "EMPTY" : "NOT EMPTY", fifo_level);
            
            if (!fifo_empty) {
                // If there's data in the FIFO, read it directly
                uint32_t fifo_data = pio_sm_get(tclk.pio, tclk.sm_rx);
                printf("DEBUG: Direct FIFO read: 0x%08X\n", fifo_data);
                
                // Flash LED to indicate data received
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                sleep_ms(50);
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            }
        }
        
        // Read received bytes with timestamp from TCLK
        uint64_t tclk_timestamp = 0;
        size_t tclk_byte_count = tclk_receive_bytes(&tclk, rx_buffer, BUFFER_SIZE, &tclk_timestamp);
        
        if (tclk_byte_count > 0) {
            // Reset empty count
            empty_count = 0;
            
            // Process the received TCLK data with timestamp
            process_received_tclk_data(rx_buffer, tclk_byte_count, tclk_timestamp);
            
            // If we got data, keep the LED on briefly to indicate activity
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(20);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        } else {
            // Count consecutive empty reads
            empty_count++;
            
            // Every 500 empty reads, print debug info
            if (empty_count % 500 == 0) {
                printf("DEBUG: No data for %d cycles\n", empty_count);
            }
            
            // If we've had a long period without data, try reconnecting
            if (empty_count > 5000) {
                printf("Warning: No data received for an extended period (%d cycles)\n", empty_count);
                
                // Flash LED to indicate reconnection attempt
                for (int i = 0; i < 3; i++) {
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                    sleep_ms(50);
                    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                    sleep_ms(50);
                }
                
                // Try to re-synchronize
                printf("Attempting to re-synchronize...\n");
                tclk_synchronize(&tclk);
                empty_count = 0;
            }
            
            // If no data available, sleep a bit
            sleep_ms(10);
        }
        
        loop_count++;
    }
    
    printf("Data processing stopped on core 1\n");
}

// Process the received TCLK data and output as JSON
void process_received_tclk_data(uint8_t* buffer, size_t byte_count, uint64_t timestamp) {
    // Process each byte as a separate TCLK event
    for (size_t i = 0; i < byte_count; i++) {
        output_tclk_event(buffer[i], timestamp);
    }
}

// Output a TCLK event to the serial port in JSON format
void output_tclk_event(uint8_t event_byte, uint64_t timestamp) {
    char json_buffer[JSON_BUFFER_SIZE];
    
    // Format the JSON event
    // Convert timestamp from microseconds to nanoseconds (multiply by 1000)
    snprintf(json_buffer, JSON_BUFFER_SIZE,
             "{\"tclk_event\":%d, \"time\":%llu}\n",
             event_byte,
             timestamp * 1000); // Convert to nanoseconds
    
    // Output the JSON event
    printf("%s", json_buffer);
}

// Process serial commands
void process_serial_command() {
    // Check if there's data available on stdin
    if (stdio_usb_connected() && getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
        char command[BUFFER_SIZE];
        int idx = 0;
        
        // Read the command
        while (idx < BUFFER_SIZE - 1) {
            int c = getchar_timeout_us(50000); // 50ms timeout
            if (c == PICO_ERROR_TIMEOUT || c == '\r' || c == '\n') {
                break;
            }
            command[idx++] = (char)c;
        }
        command[idx] = '\0';
        
        // Process the command
        if (strcmp(command, "help") == 0) {
            printf("Available commands:\n");
            printf("  help                   - Show this help message\n");
            printf("  event HEX              - Send a single byte event (0x00-0xFF)\n");
            printf("  timeline EVENT,OFFSET... - Send a timeline of events with time offsets\n");
            printf("                           Example: timeline 1F,1000000,2A,500000\n");
            printf("                           Sends event 0x1F, waits 1ms, sends 0x2A, waits 0.5ms\n");
            printf("  testloop               - Run a test loop with jumpered pins\n");
            printf("  exit                   - Exit the application\n");
        }
        else if (strncmp(command, "event ", 6) == 0) {
            const char* hex_str = command + 6;
            
            // Convert hex string to byte
            char* end;
            unsigned long value = strtoul(hex_str, &end, 16);
            
            if (*end != '\0' || value > 0xFF) {
                printf("Error: Invalid hex value. Please use format 'event XX' where XX is a hex value (00-FF)\n");
            } else {
                uint8_t byte_value = (uint8_t)value;
                printf("Sending single byte event: 0x%02X via TCLK\n", byte_value);
                
                // Send the byte using TCLK
                tclk_send_byte(&tclk, byte_value);
                output_tclk_event(byte_value, time_us_64());
            }
        }
        else if (strncmp(command, "timeline ", 9) == 0) {
            const char* timeline_str = command + 9;
            
            // Parse the timeline string (format: EVENT,OFFSET,EVENT,OFFSET,...)
            uint8_t events[32];  // Max 32 events in a timeline
            uint64_t offsets[32];
            size_t count = 0;
            
            char* token = strtok((char*)timeline_str, ",");
            while (token != NULL && count < 32) {
                // Parse event (hex value)
                char* end;
                unsigned long value = strtoul(token, &end, 16);
                
                if (*end != '\0' || value > 0xFF) {
                    printf("Error: Invalid event value '%s'. Must be a hex value (00-FF)\n", token);
                    break;
                }
                
                events[count] = (uint8_t)value;
                
                // Get the offset
                token = strtok(NULL, ",");
                if (token == NULL) {
                    printf("Error: Missing offset for event 0x%02X\n", events[count]);
                    break;
                }
                
                // Parse offset (decimal nanoseconds)
                uint64_t offset = strtoull(token, &end, 10);
                
                if (*end != '\0') {
                    printf("Error: Invalid offset value '%s'. Must be a decimal value in nanoseconds\n", token);
                    break;
                }
                
                offsets[count] = offset;
                count++;
                
                // Get the next event
                token = strtok(NULL, ",");
            }
            
            if (count > 0) {
                printf("Sending timeline with %zu events via TCLK\n", count);
                send_timeline(events, offsets, count);
            } else {
                printf("Error: No valid events in timeline\n");
            }
        }
        else if (strcmp(command, "testloop") == 0) {
            printf("Running TCLK test loop with jumpered pins...\n");
            if (tclk_run_test_loop(&tclk)) {
                printf("TCLK test loop completed successfully\n");
            } else {
                printf("TCLK test loop failed\n");
            }
        }
        else if (strcmp(command, "exit") == 0) {
            printf("Exiting...\n");
            running = false;
        }
        else if (strlen(command) > 0) {
            printf("Unknown command: %s\n", command);
            printf("Type 'help' for available commands\n");
        }
    }
}

// Send a timeline of events with specified time offsets
void send_timeline(const uint8_t* events, const uint64_t* offsets, size_t count) {
    if (!events || !offsets || count == 0) return;
    
    // Send the first event immediately
    tclk_send_byte(&tclk, events[0]);
    output_tclk_event(events[0], time_us_64());
    
    // Send the remaining events with the specified time offsets
    for (size_t i = 1; i < count; i++) {
        // Convert nanoseconds to microseconds for sleep_us (divide by 1000)
        uint64_t sleep_time_us = offsets[i-1] / 1000;
        
        // Sleep for the specified time offset
        if (sleep_time_us > 0) {
            // For longer delays, use sleep_ms to be more efficient
            if (sleep_time_us >= 1000) {
                sleep_ms(sleep_time_us / 1000);
                sleep_us(sleep_time_us % 1000);
            } else {
                sleep_us(sleep_time_us);
            }
        }
        
        // Send the next event
        tclk_send_byte(&tclk, events[i]);
        output_tclk_event(events[i], time_us_64());
    }
    
    printf("Timeline completed: %zu events sent via TCLK\n", count);
}
