/**
 * @file tclk.c
 * @brief Implementation of TCLK protocol using Raspberry Pi Pico PIO
 * 
 * This implementation separates the responsibilities:
 * - tclkIN.pio: Handles timestamps for input signals
 * - tclkDecode.pio: Converts timestamps to bits
 * - tclkEncode.pio: Converts bits to timestamps
 * - tclkOUT.pio: Outputs timestamps as signals
 * - tclk.c (this file): Handles protocol-specific operations like synchronization and parity checking
 */

#include "tclk.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Include the PIO program headers (will be generated during build)
#include "tclkIN.pio.h"
#include "tclkDecode.pio.h"
#include "tclkEncode.pio.h"
#include "tclkOUT.pio.h"

// TCLK protocol constants
#define TCLK_START_BIT 0         // Start bit is always 0
#define TCLK_DATA_BITS 8         // 8 data bits
#define TCLK_PARITY_BITS 1       // 1 parity bit (odd parity)
#define TCLK_TRAILER_BITS 2      // 2 trailer bits (always 1)
#define TCLK_FRAME_BITS (TCLK_DATA_BITS + TCLK_PARITY_BITS + TCLK_TRAILER_BITS + 1) // Total bits in a frame

// Buffer sizes
#define BIT_BUFFER_SIZE 256      // Size of the bit buffer

// Helper function to calculate odd parity for a byte
static bool calculate_odd_parity(uint8_t byte) {
    uint8_t parity = 0;
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            parity ^= 1;
        }
    }
    return parity;
}

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
    tclk->offset_in = pio_add_program(tclk->pio, &tclkIN_program);
    tclk->offset_decode = pio_add_program(tclk->pio, &tclkDecode_program);
    tclk->offset_encode = pio_add_program(tclk->pio, &tclkEncode_program);
    tclk->offset_out = pio_add_program(tclk->pio, &tclkOUT_program);
    
    // Initialize the tclkIN program
    tclkIN_program_init(tclk->pio, tclk->sm_in, tclk->offset_in, tclk->clock_in_pin);
    
    // Initialize the tclkDecode program
    tclkDecode_program_init(tclk->pio, tclk->sm_decode, tclk->offset_decode);
    
    // Initialize the tclkEncode program
    tclkEncode_program_init(tclk->pio, tclk->sm_encode, tclk->offset_encode);
    
    // Initialize the tclkOUT program
    tclkOUT_program_init(tclk->pio, tclk->sm_out, tclk->offset_out, tclk->clock_out_pin);
    
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
    
    // Set up DMA to transfer data from tclkIN to tclkDecode
    dma_channel_config c_in = dma_channel_get_default_config(tclk->dma_channel_in);
    channel_config_set_transfer_data_size(&c_in, DMA_SIZE_32);
    channel_config_set_read_increment(&c_in, false);
    channel_config_set_write_increment(&c_in, false);
    channel_config_set_dreq(&c_in, pio_get_dreq(tclk->pio, tclk->sm_in, false));
    
    dma_channel_configure(
        tclk->dma_channel_in,
        &c_in,
        &tclk->pio->txf[tclk->sm_decode],  // Write to tclkDecode TX FIFO
        &tclk->pio->rxf[tclk->sm_in],      // Read from tclkIN RX FIFO
        0,                                  // Transfer forever
        true                                // Start immediately
    );
    
    // Set up DMA to transfer data from tclkEncode to tclkOUT
    dma_channel_config c_out = dma_channel_get_default_config(tclk->dma_channel_out);
    channel_config_set_transfer_data_size(&c_out, DMA_SIZE_32);
    channel_config_set_read_increment(&c_out, false);
    channel_config_set_write_increment(&c_out, false);
    channel_config_set_dreq(&c_out, pio_get_dreq(tclk->pio, tclk->sm_encode, false));
    
    dma_channel_configure(
        tclk->dma_channel_out,
        &c_out,
        &tclk->pio->txf[tclk->sm_out],     // Write to tclkOUT TX FIFO
        &tclk->pio->rxf[tclk->sm_encode],  // Read from tclkEncode RX FIFO
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
    
    // Encode the byte into bits and send them to the PIO
    return encode_and_send_bits(tclk, byte);
}

size_t tclk_send_bytes(tclk_t* tclk, const uint8_t* buffer, size_t length) {
    if (!tclk || !tclk->is_running || !buffer || length == 0) return 0;
    
    // Send each byte individually
    size_t bytes_sent = 0;
    for (size_t i = 0; i < length; i++) {
        if (encode_and_send_bits(tclk, buffer[i])) {
            bytes_sent++;
        } else {
            // Error sending byte
            break;
        }
    }
    
    return bytes_sent;
}

// Helper function to receive bits from the PIO and assemble them into a byte
static bool receive_bits_and_assemble(tclk_t* tclk, uint8_t* byte, bool check_parity) {
    if (!tclk || !tclk->is_running || !byte) return false;
    
    bool bit_buffer[TCLK_FRAME_BITS];
    size_t bits_received = 0;
    uint8_t assembled_byte = 0;
    bool parity_bit = false;
    
    // Wait for a start bit (0)
    while (true) {
        bool bit;
        if (!tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &bit)) {
            // No data available
            return false;
        }
        
        if (bit == TCLK_START_BIT) {
            // Found start bit, break and continue to data bits
            break;
        }
    }
    
    // Read 8 data bits
    for (int i = 0; i < TCLK_DATA_BITS; i++) {
        bool bit;
        if (!tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &bit)) {
            // Not enough data available
            return false;
        }
        
        // Assemble the byte (LSB first)
        if (bit) {
            assembled_byte |= (1 << i);
        }
    }
    
    // Read parity bit
    if (!tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &parity_bit)) {
        // Not enough data available
        return false;
    }
    
    // Check parity if requested
    if (check_parity) {
        bool expected_parity = calculate_odd_parity(assembled_byte);
        if (parity_bit != expected_parity) {
            // Parity error
            return false;
        }
    }
    
    // Skip the two trailer bits (should be 1's)
    bool trailer_bit1, trailer_bit2;
    if (!tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &trailer_bit1) ||
        !tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &trailer_bit2)) {
        // Not enough data available
        return false;
    }
    
    // Check trailer bits if requested
    if (check_parity && (!trailer_bit1 || !trailer_bit2)) {
        // Invalid trailer bits
        return false;
    }
    
    // Return the assembled byte
    *byte = assembled_byte;
    return true;
}

// Helper function to encode a byte into bits and send them to the PIO
static bool encode_and_send_bits(tclk_t* tclk, uint8_t byte) {
    if (!tclk || !tclk->is_running) return false;
    
    // Calculate parity bit (odd parity)
    bool parity_bit = calculate_odd_parity(byte);
    
    // Send start bit (always 0)
    tclkEncode_send_bit(tclk->pio, tclk->sm_encode, TCLK_START_BIT);
    
    // Send 8 data bits (LSB first)
    for (int i = 0; i < TCLK_DATA_BITS; i++) {
        bool bit = (byte & (1 << i)) != 0;
        tclkEncode_send_bit(tclk->pio, tclk->sm_encode, bit);
    }
    
    // Send parity bit
    tclkEncode_send_bit(tclk->pio, tclk->sm_encode, parity_bit);
    
    // Send 2 trailer bits (always 1)
    tclkEncode_send_bit(tclk->pio, tclk->sm_encode, true);
    tclkEncode_send_bit(tclk->pio, tclk->sm_encode, true);
    
    return true;
}

// Helper function to check if the input is active and synchronized
static bool check_sync_status(tclk_t* tclk, uint32_t timeout_ms) {
    if (!tclk || !tclk->is_running) return false;
    
    // Check if we can receive valid data within the timeout period
    absolute_time_t end_time = make_timeout_time_ms(timeout_ms);
    
    while (!time_reached(end_time)) {
        // Try to receive a byte with parity checking
        uint8_t dummy_byte;
        if (receive_bits_and_assemble(tclk, &dummy_byte, true)) {
            // Successfully received a valid byte, we're synchronized
            return true;
        }
        
        // Give a small delay to avoid tight loop
        sleep_ms(1);
    }
    
    // Timeout reached without receiving valid data
    return false;
}

bool tclk_receive_byte(tclk_t* tclk, uint8_t* byte) {
    if (!tclk || !tclk->is_running || !byte) return false;
    
    // Try to receive a byte with parity checking
    bool result = receive_bits_and_assemble(tclk, byte, true);
    
    // If we couldn't receive a byte, check if we need to re-synchronize
    if (!result && tclk->is_running) {
        // Check if we've lost synchronization (no valid data for 100ms)
        if (!check_sync_status(tclk, 100)) {
            printf("TCLK synchronization lost. Attempting to re-synchronize...\n");
            // Try to re-synchronize
            tclk_synchronize(tclk);
            // Try to receive a byte again after re-synchronization
            result = receive_bits_and_assemble(tclk, byte, true);
        }
    }
    
    return result;
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
    uint16_t bit_frame = 0; // Use a 16-bit integer to store the last 16 bits (more than enough for our 12-bit frame)
    
    // Clear any existing data in the FIFOs
    pio_sm_clear_fifos(tclk->pio, tclk->sm_in);
    pio_sm_clear_fifos(tclk->pio, tclk->sm_decode);
    
    // Bit masks for checking frame components
    const uint16_t START_BIT_MASK = 0x0800;    // Bit 11 (0-indexed) - should be 0
    const uint16_t DATA_BITS_MASK = 0x07F8;    // Bits 3-10 - 8 data bits
    const uint16_t PARITY_BIT_MASK = 0x0004;   // Bit 2 - parity bit
    const uint16_t TRAILER_BITS_MASK = 0x0003; // Bits 0-1 - should be 1 (0x3)
    
    // Try to find a valid TCLK pattern (will continue indefinitely)
    while (!synchronized) {
        // Read a bit from the PIO
        bool bit;
        if (!tclkDecode_get_bit(tclk->pio, tclk->sm_decode, &bit)) {
            // No data available, wait a bit
            sleep_ms(1);
            continue;
        }
        
        // Shift the frame left by 1 and add the new bit at the end
        bit_frame = (bit_frame << 1) | (bit ? 1 : 0);
        
        // Check for a complete TCLK frame (12 bits)
        // - Start bit (0) at bit position 11
        // - 8 data bits (any value) at bit positions 3-10
        // - Parity bit (odd parity) at bit position 2
        // - 2 guaranteed '1's at bit positions 0-1
        
        // First, check if we have a valid frame structure:
        // 1. Start bit should be 0
        // 2. Trailer bits should be 1
        if (((bit_frame & START_BIT_MASK) == 0) && ((bit_frame & TRAILER_BITS_MASK) == TRAILER_BITS_MASK)) {
            // Extract the data byte (bits 3-10)
            uint8_t data_byte = (bit_frame & DATA_BITS_MASK) >> 3;
            
            // Check parity bit
            bool parity_bit = (bit_frame & PARITY_BIT_MASK) != 0;
            bool expected_parity = calculate_odd_parity(data_byte);
            
            if (parity_bit == expected_parity) {
                // We found a valid TCLK frame!
                synchronized = true;
                printf("TCLK synchronized successfully (found valid frame with data 0x%02X)\n", data_byte);
                break;
            }
        }
    }
    
    return synchronized;
}

size_t tclk_receive_bytes(tclk_t* tclk, uint8_t* buffer, size_t max_length, uint64_t* last_bit_timestamp) {
    if (!tclk || !tclk->is_running || !buffer || max_length == 0) return 0;
    
    // Get the current timestamp before receiving bytes
    uint64_t current_time = time_us_64();
    size_t bytes_received = 0;
    
    // Receive bytes one at a time with parity checking
    for (size_t i = 0; i < max_length; i++) {
        if (!receive_bits_and_assemble(tclk, &buffer[i], true)) {
            // No more data available or error
            
            // If we haven't received any bytes yet, check if we need to re-synchronize
            if (bytes_received == 0 && tclk->is_running) {
                // Check if we've lost synchronization (no valid data for 100ms)
                if (!check_sync_status(tclk, 100)) {
                    printf("TCLK synchronization lost. Attempting to re-synchronize...\n");
                    // Try to re-synchronize
                    tclk_synchronize(tclk);
                    // Try to receive this byte again after re-synchronization
                    if (receive_bits_and_assemble(tclk, &buffer[i], true)) {
                        bytes_received++;
                        continue; // Continue to the next byte
                    }
                }
            }
            
            // If we still can't receive data, break out of the loop
            break;
        }
        bytes_received++;
    }
    
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
        // Send the test byte (0x1F) using our bit-level encoding function
        encode_and_send_bits(tclk, 0x1F);
        
        // Check for received bytes
        size_t bytes_received = 0;
        for (size_t i = 0; i < sizeof(rx_buffer); i++) {
            if (!receive_bits_and_assemble(tclk, &rx_buffer[i], true)) {
                // No more data available or error
                break;
            }
            bytes_received++;
        }
        
        if (bytes_received > 0) {
            // Get current timestamp
            last_bit_timestamp = time_us_64();
            
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
