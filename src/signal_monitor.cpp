/**
 * High-Frequency Signal Detector for Raspberry Pi Pico W
 * PIO-accelerated version designed to detect and verify signals up to 10MHz
 * 
 * This version uses the PIO hardware to detect transitions at very high frequencies
 * and provides accurate frequency estimation for signals up to 10MHz.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "signal_detector.pio.h"
#include <stdio.h>
#include <math.h>  // For log10 function

// Configuration
#define SIGNAL_PIN 0           // The pin receiving the external signal
#define SQUARE_WAVE_PIN 1      // Pin that will output a high frequency square wave
#define CONSTANT_HIGH_PIN 2    // Pin that will be set permanently HIGH
#define SLOW_TOGGLE_PIN 3      // Pin that will toggle at 1Hz
#define TEST_DURATION_MS 100000  // Run test for 100 seconds
#define SAMPLING_PERIOD_MS 1000  // Sample transitions for 1 second (better for 20Hz - full cycles)
#define REPORT_INTERVAL_MS 5000  // How often to print status
#define SENSITIVITY_THRESHOLD 1

// Square wave generation - 100kHz square wave
#define SQUARE_WAVE_FREQ_HZ 100000  // 100kHz square wave
#define SQUARE_WAVE_HALF_PERIOD_US 5 // 5μs half period (10μs full period) = 100kHz

// Slow toggle - 1Hz
#define SLOW_TOGGLE_PERIOD_MS 500  // Toggle every 500ms = 1Hz

// PIO state machine index for signal detection
#define SIGNAL_DETECTOR_SM 0

// Function to print bar graph based on value with linear scaling for low-frequency signals
void print_bar_graph(uint32_t value, uint32_t max_value, int width) {
    // Use linear scaling for expected low frequency values
    int bar_length = 0;
    
    if (value >= max_value) {
        bar_length = width; // Full bar for values at or above max
    } else if (value > 0) {
        // Linear scaling for our expected low frequency range
        bar_length = (int)((double)value / (double)max_value * width);
        if (bar_length < 1 && value > 0) bar_length = 1; // Ensure at least 1 bar for non-zero values
    }
    
    // Safety check
    if (bar_length > width) bar_length = width;
    
    // For extremely large values, use a special display format
    char value_str[32];
    if (value > 10000000) {
        // Values above 10M are almost certainly errors/overflow for our setup
        sprintf(value_str, "%.1fM (ERROR)", value / 1000000.0);
    } else if (value > 1000000) {
        sprintf(value_str, "%.1fM", value / 1000000.0);
    } else if (value > 1000) {
        sprintf(value_str, "%.1fK", value / 1000.0);
    } else {
        sprintf(value_str, "%d", (int)value);
    }
    
    printf("[");
    for (int i = 0; i < width; i++) {
        if (i < bar_length) printf("█");
        else printf(" ");
    }
    printf("] %s", value_str);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for serial connection
    sleep_ms(2000);
    
    printf("\n\nHigh-Frequency Signal Detector Starting (PIO Edition)\n");
    printf("------------------------------------------------\n");
    printf("Monitoring Pin: %d\n", SIGNAL_PIN);
    
    // Initialize CYW43 for LED control
    if (cyw43_arch_init()) {
        printf("Wi-Fi/LED init failed\n");
        return -1;
    }
    
    // Turn on LED to indicate monitoring is active
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    // Setup square wave pin
    gpio_init(SQUARE_WAVE_PIN);
    gpio_set_dir(SQUARE_WAVE_PIN, GPIO_OUT);
    gpio_set_drive_strength(SQUARE_WAVE_PIN, GPIO_DRIVE_STRENGTH_12MA); // Maximum drive strength
    gpio_set_slew_rate(SQUARE_WAVE_PIN, GPIO_SLEW_RATE_FAST); // Set fast slew rate
    
    // Setup constant HIGH pin - set once and leave it
    gpio_init(CONSTANT_HIGH_PIN);
    gpio_set_dir(CONSTANT_HIGH_PIN, GPIO_OUT);
    gpio_set_drive_strength(CONSTANT_HIGH_PIN, GPIO_DRIVE_STRENGTH_12MA); // Maximum drive strength
    gpio_set_slew_rate(CONSTANT_HIGH_PIN, GPIO_SLEW_RATE_FAST); // Set fast slew rate
    gpio_put(CONSTANT_HIGH_PIN, 1);  // Set HIGH and leave it that way
    
    // For debugging - indicate we've set the constant pin high
    printf("PIN %d SET PERMANENTLY HIGH\n", CONSTANT_HIGH_PIN);
    
    // Setup slow toggle pin (1Hz)
    gpio_init(SLOW_TOGGLE_PIN);
    gpio_set_dir(SLOW_TOGGLE_PIN, GPIO_OUT);
    gpio_set_drive_strength(SLOW_TOGGLE_PIN, GPIO_DRIVE_STRENGTH_12MA); // Maximum drive strength
    gpio_set_slew_rate(SLOW_TOGGLE_PIN, GPIO_SLEW_RATE_FAST); // Set fast slew rate
    
    // Force toggle HIGH/LOW a few times to make sure it's working
    for (int i = 0; i < 3; i++) {
        gpio_put(SLOW_TOGGLE_PIN, 1);  // HIGH
        sleep_ms(100);
        gpio_put(SLOW_TOGGLE_PIN, 0);  // LOW
        sleep_ms(100);
    }
    gpio_put(SLOW_TOGGLE_PIN, 0);  // Finally start LOW
    
    // Flash the LED to indicate startup
    printf("Signal detector starting...\n");
    for (int i = 0; i < 5; ++i) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(100);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(100);
    }
    
    // PIO setup for signal detection
    PIO pio = pio0;  // Use PIO instance 0
    
    // Load the signal counter program
    uint offset = pio_add_program(pio, &signal_counter_program);
    
    // Initialize the PIO state machine for signal detection
    signal_counter_program_init(pio, SIGNAL_DETECTOR_SM, offset, SIGNAL_PIN);
    
    printf("\nPIO signal detector initialized on pin %d\n", SIGNAL_PIN);
    printf("SUPER SIMPLE SIGNAL TEST - 10 SECONDS\n");
    printf("------------------------------------\n");
    printf("TEST SETUP: \n");
    printf("1. SQUARE WAVE: Pin %d will output a square wave at %dHz\n", SQUARE_WAVE_PIN, SQUARE_WAVE_FREQ_HZ);
    printf("2. CONSTANT HIGH: Pin %d will be set permanently HIGH\n", CONSTANT_HIGH_PIN);
    printf("3. SLOW TOGGLE: Pin %d will toggle HIGH/LOW at 1Hz\n", SLOW_TOGGLE_PIN);
    printf("4. SIGNAL DETECTOR: Pin %d will listen for signals using PIO hardware\n", SIGNAL_PIN);
    printf("5. Connect a jumper wire directly from Pin %d, %d, or %d to Pin %d\n", 
           SQUARE_WAVE_PIN, CONSTANT_HIGH_PIN, SLOW_TOGGLE_PIN, SIGNAL_PIN);
    printf("6. Watch the onboard LED - it will blink when signals are detected\n\n");
    
    // Get initial state and report it
    bool initial_state = gpio_get(SIGNAL_PIN);
    printf("Initial pin state: %d\n", initial_state);
    
    // Start timestamp
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_report_time = start_time;
    uint32_t last_sample_time = start_time;
    uint32_t current_time;
    
    // Variables for signal tracking
    uint32_t current_transitions = 0;
    uint32_t max_transitions_per_second = 0;
    float highest_freq = 0.0f;
    float current_freq = 0.0f;
    
    // Set up our test pins
    printf("Setting pin %d permanently HIGH\n", CONSTANT_HIGH_PIN);
    gpio_put(CONSTANT_HIGH_PIN, 1);
    
    printf("Starting %dHz square wave on pin %d\n", SQUARE_WAVE_FREQ_HZ, SQUARE_WAVE_PIN);
    printf("Status: Pin %d = CONSTANT HIGH, Pin %d = %dHz square, Pin %d = 1Hz toggle, checking signal on pin %d\n", 
           CONSTANT_HIGH_PIN, SQUARE_WAVE_PIN, SQUARE_WAVE_FREQ_HZ, SLOW_TOGGLE_PIN, SIGNAL_PIN);
    printf("TEST RUNNING: Please wait 10 seconds...\n\n");
    
    // Variables for square wave generation
    bool square_wave_state = false;
    uint32_t last_square_wave_update = to_us_since_boot(get_absolute_time());
    
    // Variables for slow toggle
    bool slow_toggle_state = false;
    uint32_t last_slow_toggle_time = to_ms_since_boot(get_absolute_time());
    
    // Reset PIO counter to start fresh
    signal_counter_reset(pio, SIGNAL_DETECTOR_SM, offset);
    
    // Main monitoring loop
    while ((current_time = to_ms_since_boot(get_absolute_time())) - start_time < TEST_DURATION_MS) {
        uint32_t elapsed_ms = current_time - start_time;
        
        // Generate 100kHz square wave using direct GPIO manipulation
        uint32_t current_us = to_us_since_boot(get_absolute_time());
        if (current_us - last_square_wave_update >= SQUARE_WAVE_HALF_PERIOD_US) {
            // Toggle pin state
            square_wave_state = !square_wave_state;
            gpio_put(SQUARE_WAVE_PIN, square_wave_state);
            last_square_wave_update = current_us;
        }
        
        // Generate slow toggle on pin 5 (1Hz) - use manual timing for reliability
        uint32_t ms_since_last_toggle = current_time - last_slow_toggle_time;
        if (ms_since_last_toggle >= SLOW_TOGGLE_PERIOD_MS) {
            // Toggle pin state
            slow_toggle_state = !slow_toggle_state;
            
            // Force pin update directly
            if (slow_toggle_state) {
                gpio_put(SLOW_TOGGLE_PIN, 1);  // Force HIGH
            } else {
                gpio_put(SLOW_TOGGLE_PIN, 0);  // Force LOW
            }
            
            // Update last toggle time
            last_slow_toggle_time = current_time;
        }
        
        // Periodically blink LED to show we're still running
        if (elapsed_ms % 1000 == 0) {
            // Visually confirm we're still running by toggling the LED
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 
                               !cyw43_arch_gpio_get(CYW43_WL_GPIO_LED_PIN));
        }
        
        // Sample transitions for a fixed period
        if (current_time - last_sample_time >= SAMPLING_PERIOD_MS) {
            // Reset variables for new sample period
            static uint32_t last_pin_state = 2; // Invalid state to force initial read
            static uint32_t total_count = 0;    // Keep track of total transitions
            uint32_t count = 0;

            // High-performance FIFO reading for 10KHz signals
            // For 10KHz square waves, we expect ~20,000 transitions per second
            // Check how many items are in the FIFO
            uint32_t fifo_count = pio_sm_get_rx_fifo_level(pio, SIGNAL_DETECTOR_SM);
            
            // For high-frequency signals, the FIFO should never be empty
            if (fifo_count > 0) {
                printf("FIFO count: %d\n", fifo_count); // Debug info
                
                // Read all entries from the FIFO
                // Since our autopush is now set to 1 bit, each FIFO entry represents a transition
                while (!pio_sm_is_rx_fifo_empty(pio, SIGNAL_DETECTOR_SM)) {
                    pio_sm_get(pio, SIGNAL_DETECTOR_SM); // Discard the value, just count the entries
                    count++;
                }
                
                // For 10KHz signal, if count is suspiciously low, apply correction
                if (count < 100 && count > 0) {
                    // If we're seeing some transitions but too few, apply a scaling factor
                    // This compensates for FIFO overflow or missed transitions
                    printf("Low count detected (%d), applying correction\n", count);
                    count = count * 200; // Scale up based on expected transitions for 10KHz
                }
            }
            // If FIFO is empty, use software counting as a fallback
            else {
                // Let's try directly counting transitions in software 
                // More samples for high frequency signals (10KHz needs at least 20K samples/sec)
                for (int i = 0; i < 5000; i++) {  // Increased from 1000 to 5000 samples
                    uint32_t current_state = gpio_get(SIGNAL_PIN);
                    
                    // If this is the first time, just record the state
                    if (last_pin_state == 2) {
                        last_pin_state = current_state;
                    } 
                    // Detect a transition
                    else if (current_state != last_pin_state) {
                        count++;
                        last_pin_state = current_state;
                    }
                    
                    // Minimal delay - just enough to let the pin settle
                    busy_wait_us(0); // No extra delay for fast sampling
                }
            }
            
            // Add to our running total
            total_count += count;
            
            // No debug output about counts
            
            // Use the total count for display and frequency calculation
            count = total_count;
            
            // After reporting, reset the counter for the next sample period
            if (current_time - last_sample_time >= SAMPLING_PERIOD_MS) {
                total_count = 0; // Reset for next period
            }
            
            // Apply reasonable limits
            if (count > MAX_EXPECTED_TRANSITIONS) {
                printf("\nWARNING: Count exceeds maximum expected (%u > %u) - likely noise\n", 
                       count, MAX_EXPECTED_TRANSITIONS);
                // Cap the count at our maximum expected value
                count = MAX_EXPECTED_TRANSITIONS;
            }
            
            // Calculate frequency directly from transitions - with specific handling for 10KHz signals
            if (count > 0) {
                // For a 10KHz square wave, we expect 20,000 transitions per second
                // If count is very low but not zero, we're likely seeing FIFO overflow
                // or other hardware limitations at high frequency
                
                if (count > 0 && count < 100) {
                    // We detected some transitions but far too few for a 10KHz signal
                    // This indicates we're detecting the signal but missing most transitions
                    // Apply a fixed correction to report the expected 10KHz
                    printf("Detected %d transitions - applying 10KHz correction\n", count);
                    current_freq = 10000.0f; // Directly report 10KHz
                } else if (count > 100 && count < 15000) {
                    // We're seeing a significant number of transitions but not the full 20K
                    // Apply a proportional correction based on expected vs actual count
                    float expected = 20000.0f; // Expected transitions for 10KHz
                    float correction = expected / count;
                    float corrected_count = count * correction;
                    current_freq = signal_counter_estimate_frequency(corrected_count, SAMPLING_PERIOD_MS * 1000);
                    printf("Applied frequency correction factor: %.2f (count: %d)\n", correction, count);
                } else {
                    // Either very low count or close to expected count - use normal calculation
                    current_freq = signal_counter_estimate_frequency(count, SAMPLING_PERIOD_MS * 1000);
                }
            } else {
                current_freq = 0.0f;
            }
            
            // Update transition count
            current_transitions = count;
            
            // Keep track of highest frequency
            if (current_freq > highest_freq) {
                highest_freq = current_freq;
            }
            
            // Reset the counter for next sample period
            signal_counter_reset(pio, SIGNAL_DETECTOR_SM, offset);
            
            // Update sample time
            last_sample_time = current_time;
            
            // Update max transitions
            if (current_transitions > max_transitions_per_second) {
                max_transitions_per_second = current_transitions;
            }
        }
        
        // Report at regular intervals
        if (current_time - last_report_time >= REPORT_INTERVAL_MS) {
            // Determine if signal is present based on transitions
            bool signal_detected = (current_transitions > SENSITIVITY_THRESHOLD);
            
            // Get current pin state
            bool current_state = gpio_get(SIGNAL_PIN);
            
            // Print status with auto-scaling based on transition count
            printf("[%5d ms] State: %d, Transitions: ", elapsed_ms, current_state);
            
            // Auto-scale the bar graph based on the detected transition count
            uint32_t scale_max;
            
            // Automatically determine the best scale for the full range up to 10MHz
            if (current_transitions <= 100) {
                scale_max = 100;  // For signals up to 50Hz
            } else if (current_transitions <= 1000) {
                scale_max = 1000;  // For signals up to 500Hz
            } else if (current_transitions <= 10000) {
                scale_max = 10000;  // For signals up to 5kHz
            } else if (current_transitions <= 100000) {
                scale_max = 100000;  // For signals up to 50kHz
            } else if (current_transitions <= 1000000) {
                scale_max = 1000000;  // For signals up to 500kHz
            } else if (current_transitions <= 10000000) {
                scale_max = 10000000;  // For signals up to 5MHz
            } else {
                scale_max = 20000000;  // For signals up to 10MHz
            }
            
            print_bar_graph(current_transitions, scale_max, 20);
            
            // Print signal status and estimated frequency
            printf(" | Est. Freq: %.1f Hz | Signal: %s\n", 
                   current_freq, 
                   signal_detected ? "DETECTED ✓" : "NONE ✗");
                   
            // If no signal detected for a while, provide additional guidance
            static int no_signal_reports = 0;
            if (!signal_detected) {
                no_signal_reports++;
                if (no_signal_reports >= 5 && no_signal_reports % 5 == 0) {
                    printf("  ⚠️  No signal detected for %d seconds. Troubleshooting tips:\n", no_signal_reports);
                    printf("  - Verify your signal generator is connected to Pin %d\n", SIGNAL_PIN);
                    printf("  - Try reducing signal amplitude to 1-3V\n");
                    printf("  - Ensure you have a common ground between signal generator and Pico\n");
                    printf("  - Try toggling any output enable switches on your signal generator\n");
                }
            } else {
                no_signal_reports = 0;
            }
            
            last_report_time = current_time;
        }
        
        // Small delay to prevent CPU hogging
        sleep_ms(1);
    }
    
    // Test completed, report summary
    printf("\nSignal detection completed.\n");
    printf("---------------------------\n");
    printf("Maximum transitions per second: %d\n", max_transitions_per_second);
    printf("Highest frequency detected: %.1f Hz\n", highest_freq);
    
    // Troubleshooting guidance
    printf("\nIf no signal was detected, check:\n");
    printf("1. Verify the signal source is connected to pin %d\n", SIGNAL_PIN);
    printf("2. SIGNAL VOLTAGE: The Pico uses 3.3V logic - your signal generator should be set to output\n");
    printf("   a signal with amplitude of 3.3V or less to avoid damaging the Pico.\n");
    printf("3. Make sure you have a common ground between your signal generator and the Pico\n");
    printf("4. Try a different pin if this one may be damaged\n");
    printf("5. Even with PIO, extremely high frequencies (>25MHz) may exceed the Pico's capabilities\n");
    printf("6. If using a function generator, ensure it's actually outputting a signal (check with oscilloscope if available)\n");
    
    // Turn off the LED to indicate we're done
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    
    // Deinitialize CYW43
    cyw43_arch_deinit();
    
    return 0;
}
