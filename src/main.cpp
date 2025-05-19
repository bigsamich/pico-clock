/**
 * @file main.cpp
 * @brief Main application for the Raspberry Pi Pico TCLK protocol implementation
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "tclk.h"

// Configuration
#define CLOCK_INPUT_PIN 15   // GPIO pin for clock input
#define CLOCK_OUTPUT_PIN 16  // GPIO pin for clock output
#define BUFFER_SIZE 256      // Size of the buffer for data bytes
#define JSON_BUFFER_SIZE 128 // Size of the JSON buffer

// Function prototypes
void core1_entry();
void process_received_data(uint8_t* buffer, size_t byte_count, uint64_t timestamp);
void output_tclk_event(uint8_t event_byte, uint64_t timestamp);
void process_serial_command();
void send_timeline(const uint8_t* events, const uint64_t* offsets, size_t count);

// Global variables
static tclk_t tclk;
static volatile bool running = true;

int main() {
    // Initialize stdio
    stdio_init_all();
    
    printf("Raspberry Pi Pico TCLK Protocol Implementation\n");
    printf("--------------------------------------------\n");
    
    // Initialize the TCLK interface
    if (!tclk_init(&tclk, pio0, CLOCK_INPUT_PIN, CLOCK_OUTPUT_PIN)) {
        printf("Failed to initialize TCLK interface\n");
        return 1;
    }
    
    printf("TCLK interface initialized (IN: pin %d, OUT: pin %d)\n", 
           CLOCK_INPUT_PIN, CLOCK_OUTPUT_PIN);
    
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
    
    while (running) {
        // Read received bytes with timestamp
        uint64_t last_bit_timestamp = 0;
        size_t byte_count = tclk_receive_bytes(&tclk, rx_buffer, BUFFER_SIZE, &last_bit_timestamp);
        
        if (byte_count > 0) {
            // Process the received data with timestamp
            process_received_data(rx_buffer, byte_count, last_bit_timestamp);
        } else {
            // No data available, sleep a bit
            sleep_ms(10);
        }
    }
    
    printf("Data processing stopped on core 1\n");
}

// Process the received data and output as JSON
void process_received_data(uint8_t* buffer, size_t byte_count, uint64_t timestamp) {
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
                printf("Sending single byte event: 0x%02X\n", byte_value);
                
                // Send the byte
                tclk_send_byte(&tclk, byte_value);
                
                // Output TCLK event for the sent data
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
                printf("Sending timeline with %zu events\n", count);
                send_timeline(events, offsets, count);
            } else {
                printf("Error: No valid events in timeline\n");
            }
        }
        else if (strcmp(command, "testloop") == 0) {
            printf("Running TCLK test loop with jumpered pins...\n");
            if (tclk_run_test_loop(&tclk)) {
                printf("Test loop completed successfully\n");
            } else {
                printf("Test loop failed\n");
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
    
    printf("Timeline completed: %zu events sent\n", count);
}
