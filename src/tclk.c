/**
 * @file tclk.c
 * @brief Implementation of TCLK protocol using Raspberry Pi Pico PIO
 */

#include "tclk.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Include the PIO program headers (will be generated during build)
#include "clockIN.pio.h"
#include "clockDecode.pio.h"
#include "clockEncode.pio.h"
#include "clockOUT.pio.h"

bool tclk_init(tclk_t* tclk, PIO pio_instance, uint clock_in_pin, uint clock_out_pin) {
    if (!tclk) return false;
    
    // Initialize the TCLK structure
    tclk->pio = pio_instance;
    tclk->clock_in_pin = clock_in_pin;
    tclk->clock_out_pin = clock_out_pin;
    tclk->is_running = false;
    
    // Claim state machines
    tclk->sm_in = pio_claim_unused_sm(tclk->pio, true);
    if (tclk->sm_in == -1) return false;
    
    tclk->sm_decode = pio_claim_unused_sm(tclk->pio, true);
    if (tclk->sm_decode == -1) {
        pio_sm_unclaim(tclk->pio, tclk->sm_in);
        return false;
    }
    
    tclk->sm_encode = pio_claim_unused_sm(tclk->pio, true);
    if (tclk->sm_encode == -1) {
        pio_sm_unclaim(tclk->pio, tclk->sm_in);
        pio_sm_unclaim(tclk->pio, tclk->sm_decode);
        return false;
    }
    
    tclk->sm_out = pio_claim_unused_sm(tclk->pio, true);
    if (tclk->sm_out == -1) {
        pio_sm_unclaim(tclk->pio, tclk->sm_in);
        pio_sm_unclaim(tclk->pio, tclk->sm_decode);
        pio_sm_unclaim(tclk->pio, tclk->sm_encode);
        return false;
    }
    
    // Claim DMA channels
    tclk->dma_channel_in = dma_claim_unused_channel(true);
    if (tclk->dma_channel_in == -1) {
        pio_sm_unclaim(tclk->pio, tclk->sm_in);
        pio_sm_unclaim(tclk->pio, tclk->sm_decode);
        pio_sm_unclaim(tclk->pio, tclk->sm_encode);
        pio_sm_unclaim(tclk->pio, tclk->sm_out);
        return false;
    }
    
    tclk->dma_channel_out = dma_claim_unused_channel(true);
    if (tclk->dma_channel_out == -1) {
        pio_sm_unclaim(tclk->pio, tclk->sm_in);
        pio_sm_unclaim(tclk->pio, tclk->sm_decode);
        pio_sm_unclaim(tclk->pio, tclk->sm_encode);
        pio_sm_unclaim(tclk->pio, tclk->sm_out);
        dma_channel_unclaim(tclk->dma_channel_in);
        return false;
    }
    
    // Load PIO programs
    tclk->offset_in = pio_add_program(tclk->pio, &clockIN_program);
    tclk->offset_decode = pio_add_program(tclk->pio, &clockDecode_program);
    tclk->offset_encode = pio_add_program(tclk->pio, &clockEncode_program);
    tclk->offset_out = pio_add_program(tclk->pio, &clockOUT_program);
    
    // Initialize the clockIN program
    clockIN_program_init(tclk->pio, tclk->sm_in, tclk->offset_in, tclk->clock_in_pin);
    
    // Initialize the clockDecode program
    clockDecode_program_init(tclk->pio, tclk->sm_decode, tclk->offset_decode);
    
    // Initialize the clockEncode program
    clockEncode_program_init(tclk->pio, tclk->sm_encode, tclk->offset_encode);
    
    // Initialize the clockOUT program
    clockOUT_program_init(tclk->pio, tclk->sm_out, tclk->offset_out, tclk->clock_out_pin);
    
    return true;
}

bool tclk_start(tclk_t* tclk) {
    if (!tclk || tclk->is_running) return false;
    
    // Clear any existing data in the FIFOs to ensure clean start
    pio_sm_clear_fifos(tclk->pio, tclk->sm_in);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_decode);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_encode);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_out);
    
    // Enable state machines
    pio_sm_set_enabled(tclk->pio, tclk->sm_in, true);
    pio_sm_set_enabled(tclk->pio, tclk->sm_decode, true);
    pio_sm_set_enabled(tclk->pio, tclk->sm_encode, true);
    pio_sm_set_enabled(tclk->pio, tclk->sm_out, true);
    
    // Set up DMA to transfer data from clockIN to clockDecode
    dma_channel_config c_in = dma_channel_get_default_config(tclk->dma_channel_in);
    channel_config_set_transfer_data_size(&c_in, DMA_SIZE_32);
    channel_config_set_read_increment(&c_in, false);
    channel_config_set_write_increment(&c_in, false);
    channel_config_set_dreq(&c_in, pio_get_dreq(tclk->pio, tclk->sm_in, false));
    
    dma_channel_configure(
        tclk->dma_channel_in,
        &c_in,
        &tclk->pio->txf[tclk->sm_decode],  // Write to clockDecode TX FIFO
        &tclk->pio->rxf[tclk->sm_in],      // Read from clockIN RX FIFO
        0,                                  // Transfer forever
        true                                // Start immediately
    );
    
    // Set up DMA to transfer data from clockEncode to clockOUT
    dma_channel_config c_out = dma_channel_get_default_config(tclk->dma_channel_out);
    channel_config_set_transfer_data_size(&c_out, DMA_SIZE_32);
    channel_config_set_read_increment(&c_out, false);
    channel_config_set_write_increment(&c_out, false);
    channel_config_set_dreq(&c_out, pio_get_dreq(tclk->pio, tclk->sm_encode, false));
    
    dma_channel_configure(
        tclk->dma_channel_out,
        &c_out,
        &tclk->pio->txf[tclk->sm_out],     // Write to clockOUT TX FIFO
        &tclk->pio->rxf[tclk->sm_encode],  // Read from clockEncode RX FIFO
        0,                                  // Transfer forever
        true                                // Start immediately
    );
    
    tclk->is_running = true;
    return true;
}

void tclk_stop(tclk_t* tclk) {
    if (!tclk || !tclk->is_running) return;
    
    // Disable state machines
    pio_sm_set_enabled(tclk->pio, tclk->sm_in, false);
    pio_sm_set_enabled(tclk->pio, tclk->sm_decode, false);
    pio_sm_set_enabled(tclk->pio, tclk->sm_encode, false);
    pio_sm_set_enabled(tclk->pio, tclk->sm_out, false);
    
    // Abort DMA transfers
    dma_channel_abort(tclk->dma_channel_in);
    dma_channel_abort(tclk->dma_channel_out);
    
    tclk->is_running = false;
}

bool tclk_send_byte(tclk_t* tclk, uint8_t byte) {
    if (!tclk || !tclk->is_running) return false;
    
    // Send the byte to the clockEncode state machine
    clockEncode_send_byte(tclk->pio, tclk->sm_encode, byte);
    return true;
}

size_t tclk_send_bytes(tclk_t* tclk, const uint8_t* buffer, size_t length) {
    if (!tclk || !tclk->is_running || !buffer || length == 0) return 0;
    
    // Send the bytes to the clockEncode state machine
    clockEncode_send_bytes(tclk->pio, tclk->sm_encode, buffer, length);
    return length;
}

bool tclk_receive_byte(tclk_t* tclk, uint8_t* byte) {
    if (!tclk || !tclk->is_running || !byte) return false;
    
    // Receive a byte from the clockDecode state machine
    return clockDecode_get_byte(tclk->pio, tclk->sm_decode, byte);
}

// Define the TCLK pattern to look for during synchronization
// TCLK uses a start bit (0), followed by 8 data bits, a parity bit, and two guaranteed '1's
// For synchronization, we'll look for the pattern of a '0' followed by two '1's
// This is the end of one byte and the start of the next

bool tclk_synchronize(tclk_t* tclk) {
    if (!tclk || !tclk->is_running) return false;
    
    printf("Synchronizing TCLK interface...\n");
    printf("Waiting for valid TCLK signal (this will continue indefinitely until synchronized)...\n");
    
    // Variables to track synchronization
    bool synchronized = false;
    uint8_t bit_buffer[32]; // Buffer to store bits for pattern matching
    size_t bit_count = 0;
    
    // Clear any existing data in the FIFOs
    pio_sm_clear_fifos(tclk->pio, tclk->sm_in);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_decode);
    
    // Try to find a valid TCLK pattern (will continue indefinitely)
    while (!synchronized) {
        // Read a bit from the PIO
        bool bit;
        if (pio_sm_is_rx_fifo_empty(tclk->pio, tclk->sm_decode)) {
            // No data available, wait a bit
            sleep_ms(1);
            continue;
        }
        
        // Get a bit from the FIFO
        uint32_t data = pio_sm_get(tclk->pio, tclk->sm_decode);
        bit = (data & 0x01) != 0;
        
        // Add the bit to our buffer
        if (bit_count < sizeof(bit_buffer)) {
            bit_buffer[bit_count++] = bit ? 1 : 0;
        } else {
            // Shift the buffer to make room for the new bit
            for (size_t i = 0; i < sizeof(bit_buffer) - 1; i++) {
                bit_buffer[i] = bit_buffer[i + 1];
            }
            bit_buffer[sizeof(bit_buffer) - 1] = bit ? 1 : 0;
        }
        
        // Check for the pattern: 0 (start bit) followed by two 1s (end of previous byte)
        // This is a simplified pattern that should be enough to synchronize
        if (bit_count >= 3) {
            size_t pattern_start = bit_count - 3;
            if (bit_buffer[pattern_start] == 0 && 
                bit_buffer[pattern_start + 1] == 1 && 
                bit_buffer[pattern_start + 2] == 1) {
                synchronized = true;
                printf("TCLK synchronized successfully\n");
                break;
            }
        }
    }
    
    // We should never reach here unless synchronized is true
    printf("TCLK synchronized successfully\n");
    
    return synchronized;
}

size_t tclk_receive_bytes(tclk_t* tclk, uint8_t* buffer, size_t max_length, uint64_t* last_bit_timestamp) {
    if (!tclk || !tclk->is_running || !buffer || max_length == 0) return 0;
    
    // Get the current timestamp before receiving bytes
    uint64_t current_time = time_us_64();
    
    // Receive bytes from the clockDecode state machine
    size_t bytes_received = clockDecode_get_bytes(tclk->pio, tclk->sm_decode, buffer, max_length);
    
    // If requested, store the timestamp of the last bit
    if (last_bit_timestamp != NULL && bytes_received > 0) {
        *last_bit_timestamp = current_time;
    }
    
    return bytes_received;
}

void tclk_deinit(tclk_t* tclk) {
    if (!tclk) return;
    
    // Stop TCLK if running
    if (tclk->is_running) {
        tclk_stop(tclk);
    }
    
    // Free resources
    pio_sm_unclaim(tclk->pio, tclk->sm_in);
    pio_sm_unclaim(tclk->pio, tclk->sm_decode);
    pio_sm_unclaim(tclk->pio, tclk->sm_encode);
    pio_sm_unclaim(tclk->pio, tclk->sm_out);
    dma_channel_unclaim(tclk->dma_channel_in);
    dma_channel_unclaim(tclk->dma_channel_out);
    
    // Clear the structure
    tclk->pio = NULL;
    tclk->sm_in = 0;
    tclk->sm_decode = 0;
    tclk->sm_encode = 0;
    tclk->sm_out = 0;
    tclk->offset_in = 0;
    tclk->offset_decode = 0;
    tclk->offset_encode = 0;
    tclk->offset_out = 0;
    tclk->clock_in_pin = 0;
    tclk->clock_out_pin = 0;
    tclk->dma_channel_in = 0;
    tclk->dma_channel_out = 0;
    tclk->is_running = false;
}

bool tclk_run_test_loop(tclk_t* tclk) {
    if (!tclk || !tclk->is_running) return false;
    
    printf("Starting TCLK test loop with jumpered pins...\n");
    printf("Encoding 0x1F (15 Hz event) for 5 seconds and decoding in real time\n");
    
    // Make sure the pins are jumpered
    printf("Ensure that pin %d (output) is jumpered to pin %d (input)\n", 
           tclk->clock_out_pin, tclk->clock_in_pin);
    
    // Clear any existing data in the FIFOs
    pio_sm_clear_fifos(tclk->pio, tclk->sm_in);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_decode);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_encode);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_out);
    
    // Buffer for received bytes
    uint8_t rx_buffer[256];
    uint64_t last_bit_timestamp = 0;
    size_t total_bytes_received = 0;
    
    // Start time for the 5-second test
    absolute_time_t end_time = make_timeout_time_ms(5000);
    
    // Send the 0x1F byte repeatedly at 15 Hz (approximately every 67 ms)
    while (!time_reached(end_time)) {
        // Send the test byte (0x1F)
        tclk_send_byte(tclk, 0x1F);
        
        // Check for received bytes
        size_t bytes_received = tclk_receive_bytes(tclk, rx_buffer, sizeof(rx_buffer), &last_bit_timestamp);
        
        if (bytes_received > 0) {
            // Print the received bytes in the simplified JSON format
            for (size_t i = 0; i < bytes_received; i++) {
                printf("{\"tclk_event\":%d, \"time\":%llu}\n", 
                       rx_buffer[i], 
                       last_bit_timestamp * 1000); // Convert to nanoseconds
            }
            
            total_bytes_received += bytes_received;
        }
        
        // Wait approximately 67 ms for 15 Hz
        sleep_ms(67);
    }
    
    printf("TCLK test loop completed\n");
    printf("Total bytes received: %zu\n", total_bytes_received);
    
    // Check if we received any bytes (test success)
    if (total_bytes_received > 0) {
        printf("Test PASSED: Successfully encoded and decoded data through jumpered pins\n");
        return true;
    } else {
        printf("Test FAILED: No bytes were received. Check the jumper connection\n");
        return false;
    }
}
