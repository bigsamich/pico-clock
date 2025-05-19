/**
 * @file mdat.c
 * @brief MDAT protocol implementation using Raspberry Pi Pico PIO
 */

#include "mdat.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "mdatIN.pio.h"
#include "mdatDecode.pio.h"
#include "mdatEncode.pio.h"
#include "mdatOUT.pio.h"

// Constants
#define MDAT_BUFFER_SIZE 256
#define MDAT_SYNC_TIMEOUT_MS 5000
#define MDAT_SYNC_PATTERN 0x1F
#define MDAT_SYNC_COUNT 5

// Static buffer for DMA transfers
static uint8_t mdat_dma_buffer[MDAT_BUFFER_SIZE];

/**
 * @brief Initialize the MDAT interface
 */
bool mdat_init(mdat_t* mdat, PIO pio_instance, uint clock_in_pin, uint clock_out_pin) {
    if (!mdat || (pio_instance != pio0 && pio_instance != pio1)) {
        return false;
    }
    
    // Initialize the structure
    mdat->pio = pio_instance;
    mdat->clock_in_pin = clock_in_pin;
    mdat->clock_out_pin = clock_out_pin;
    mdat->is_running = false;
    
    // Claim state machines and DMA channels
    mdat->sm_in = pio_claim_unused_sm(mdat->pio, true);
    mdat->sm_decode = pio_claim_unused_sm(mdat->pio, true);
    mdat->sm_encode = pio_claim_unused_sm(mdat->pio, true);
    mdat->sm_out = pio_claim_unused_sm(mdat->pio, true);
    
    if (mdat->sm_in == -1 || mdat->sm_decode == -1 || 
        mdat->sm_encode == -1 || mdat->sm_out == -1) {
        // Failed to claim state machines
        mdat_deinit(mdat);
        return false;
    }
    
    // Claim DMA channels
    mdat->dma_channel_in = dma_claim_unused_channel(true);
    mdat->dma_channel_out = dma_claim_unused_channel(true);
    
    if (mdat->dma_channel_in == -1 || mdat->dma_channel_out == -1) {
        // Failed to claim DMA channels
        mdat_deinit(mdat);
        return false;
    }
    
    // Load PIO programs
    mdat->offset_in = pio_add_program(mdat->pio, &mdatIN_program);
    mdat->offset_decode = pio_add_program(mdat->pio, &mdatDecode_program);
    mdat->offset_encode = pio_add_program(mdat->pio, &mdatEncode_program);
    mdat->offset_out = pio_add_program(mdat->pio, &mdatOUT_program);
    
    // Initialize PIO programs
    mdatIN_program_init(mdat->pio, mdat->sm_in, mdat->offset_in, mdat->clock_in_pin);
    mdatDecode_program_init(mdat->pio, mdat->sm_decode, mdat->offset_decode);
    mdatEncode_program_init(mdat->pio, mdat->sm_encode, mdat->offset_encode);
    mdatOUT_program_init(mdat->pio, mdat->sm_out, mdat->offset_out, mdat->clock_out_pin);
    
    return true;
}

/**
 * @brief Start the MDAT interface
 */
bool mdat_start(mdat_t* mdat) {
    if (!mdat || mdat->is_running) {
        return false;
    }
    
    // Configure DMA for input
    dma_channel_config dma_in_config = dma_channel_get_default_config(mdat->dma_channel_in);
    channel_config_set_read_increment(&dma_in_config, false);
    channel_config_set_write_increment(&dma_in_config, true);
    channel_config_set_dreq(&dma_in_config, pio_get_dreq(mdat->pio, mdat->sm_decode, false));
    channel_config_set_transfer_data_size(&dma_in_config, DMA_SIZE_8);
    
    dma_channel_configure(
        mdat->dma_channel_in,
        &dma_in_config,
        mdat_dma_buffer,                  // Destination
        &mdat->pio->rxf[mdat->sm_decode], // Source
        MDAT_BUFFER_SIZE,                 // Transfer count
        false                             // Don't start yet
    );
    
    // Configure DMA for output
    dma_channel_config dma_out_config = dma_channel_get_default_config(mdat->dma_channel_out);
    channel_config_set_read_increment(&dma_out_config, true);
    channel_config_set_write_increment(&dma_out_config, false);
    channel_config_set_dreq(&dma_out_config, pio_get_dreq(mdat->pio, mdat->sm_encode, true));
    channel_config_set_transfer_data_size(&dma_out_config, DMA_SIZE_8);
    
    dma_channel_configure(
        mdat->dma_channel_out,
        &dma_out_config,
        &mdat->pio->txf[mdat->sm_encode], // Destination
        mdat_dma_buffer,                  // Source
        0,                                // Transfer count (set when sending)
        false                             // Don't start yet
    );
    
    // Start the state machines
    pio_sm_set_enabled(mdat->pio, mdat->sm_in, true);
    pio_sm_set_enabled(mdat->pio, mdat->sm_decode, true);
    pio_sm_set_enabled(mdat->pio, mdat->sm_encode, true);
    pio_sm_set_enabled(mdat->pio, mdat->sm_out, true);
    
    // Start DMA for input
    dma_channel_start(mdat->dma_channel_in);
    
    mdat->is_running = true;
    return true;
}

/**
 * @brief Synchronize the MDAT interface with the incoming bit stream
 */
bool mdat_synchronize(mdat_t* mdat) {
    if (!mdat) {
        return false;
    }
    
    // Wait for a valid MDAT pattern
    // In a real implementation, this would involve more sophisticated
    // pattern detection and synchronization
    
    // For simplicity, we'll just wait for a few seconds
    sleep_ms(100);
    
    return true;
}

/**
 * @brief Stop the MDAT interface
 */
void mdat_stop(mdat_t* mdat) {
    if (!mdat || !mdat->is_running) {
        return;
    }
    
    // Stop DMA channels
    dma_channel_abort(mdat->dma_channel_in);
    dma_channel_abort(mdat->dma_channel_out);
    
    // Stop state machines
    pio_sm_set_enabled(mdat->pio, mdat->sm_in, false);
    pio_sm_set_enabled(mdat->pio, mdat->sm_decode, false);
    pio_sm_set_enabled(mdat->pio, mdat->sm_encode, false);
    pio_sm_set_enabled(mdat->pio, mdat->sm_out, false);
    
    mdat->is_running = false;
}

/**
 * @brief Send a byte over the MDAT interface
 */
bool mdat_send_byte(mdat_t* mdat, uint8_t byte) {
    if (!mdat || !mdat->is_running) {
        return false;
    }
    
    // Wait for space in the FIFO
    while (pio_sm_is_tx_fifo_full(mdat->pio, mdat->sm_encode)) {
        tight_loop_contents();
    }
    
    // Send the byte
    pio_sm_put(mdat->pio, mdat->sm_encode, byte);
    
    return true;
}

/**
 * @brief Send multiple bytes over the MDAT interface
 */
size_t mdat_send_bytes(mdat_t* mdat, const uint8_t* buffer, size_t length) {
    if (!mdat || !mdat->is_running || !buffer || length == 0) {
        return 0;
    }
    
    // Copy data to DMA buffer
    size_t bytes_to_send = length > MDAT_BUFFER_SIZE ? MDAT_BUFFER_SIZE : length;
    memcpy(mdat_dma_buffer, buffer, bytes_to_send);
    
    // Configure and start DMA transfer
    dma_channel_set_read_addr(mdat->dma_channel_out, mdat_dma_buffer, false);
    dma_channel_set_trans_count(mdat->dma_channel_out, bytes_to_send, true);
    
    // Wait for DMA to complete
    dma_channel_wait_for_finish_blocking(mdat->dma_channel_out);
    
    return bytes_to_send;
}

/**
 * @brief Receive a byte from the MDAT interface
 */
bool mdat_receive_byte(mdat_t* mdat, uint8_t* byte) {
    if (!mdat || !mdat->is_running || !byte) {
        return false;
    }
    
    // Check if there's data available
    if (pio_sm_is_rx_fifo_empty(mdat->pio, mdat->sm_decode)) {
        return false;
    }
    
    // Read the byte
    *byte = pio_sm_get(mdat->pio, mdat->sm_decode) & 0xFF;
    return true;
}

/**
 * @brief Receive multiple bytes from the MDAT interface
 */
size_t mdat_receive_bytes(mdat_t* mdat, uint8_t* buffer, size_t max_length, uint64_t* last_bit_timestamp) {
    if (!mdat || !mdat->is_running || !buffer || max_length == 0) {
        return 0;
    }
    
    size_t bytes_received = 0;
    uint64_t timestamp = 0;
    
    // Check if DMA has received any data
    size_t dma_count = MDAT_BUFFER_SIZE - dma_channel_hw_addr(mdat->dma_channel_in)->transfer_count;
    
    if (dma_count > 0) {
        // Copy data from DMA buffer to output buffer
        size_t bytes_to_copy = dma_count > max_length ? max_length : dma_count;
        memcpy(buffer, mdat_dma_buffer, bytes_to_copy);
        bytes_received = bytes_to_copy;
        
        // Reset DMA for next transfer
        dma_channel_abort(mdat->dma_channel_in);
        dma_channel_configure(
            mdat->dma_channel_in,
            NULL,                             // Keep existing config
            mdat_dma_buffer,                  // Destination
            &mdat->pio->rxf[mdat->sm_decode], // Source
            MDAT_BUFFER_SIZE,                 // Transfer count
            true                              // Start immediately
        );
        
        // Get current timestamp
        timestamp = time_us_64();
    }
    
    // Store timestamp if requested
    if (last_bit_timestamp) {
        *last_bit_timestamp = timestamp;
    }
    
    return bytes_received;
}

/**
 * @brief Clean up the MDAT interface and free resources
 */
void mdat_deinit(mdat_t* mdat) {
    if (!mdat) {
        return;
    }
    
    // Stop if running
    if (mdat->is_running) {
        mdat_stop(mdat);
    }
    
    // Unclaim state machines
    if (mdat->sm_in != -1) {
        pio_sm_unclaim(mdat->pio, mdat->sm_in);
    }
    if (mdat->sm_decode != -1) {
        pio_sm_unclaim(mdat->pio, mdat->sm_decode);
    }
    if (mdat->sm_encode != -1) {
        pio_sm_unclaim(mdat->pio, mdat->sm_encode);
    }
    if (mdat->sm_out != -1) {
        pio_sm_unclaim(mdat->pio, mdat->sm_out);
    }
    
    // Unclaim DMA channels
    if (mdat->dma_channel_in != -1) {
        dma_channel_unclaim(mdat->dma_channel_in);
    }
    if (mdat->dma_channel_out != -1) {
        dma_channel_unclaim(mdat->dma_channel_out);
    }
    
    // Reset structure
    mdat->sm_in = -1;
    mdat->sm_decode = -1;
    mdat->sm_encode = -1;
    mdat->sm_out = -1;
    mdat->dma_channel_in = -1;
    mdat->dma_channel_out = -1;
    mdat->is_running = false;
}

/**
 * @brief Run a test loop for MDAT with jumpered pins
 */
bool mdat_run_test_loop(mdat_t* mdat) {
    if (!mdat || !mdat->is_running) {
        return false;
    }
    
    printf("Running MDAT test loop with jumpered pins...\n");
    printf("Sending 0x1F events at 15 Hz for 5 seconds\n");
    
    // Send test pattern
    uint8_t test_byte = MDAT_SYNC_PATTERN;
    uint32_t start_time = time_us_32();
    uint32_t end_time = start_time + 5000000; // 5 seconds
    uint32_t next_send_time = start_time;
    uint32_t interval = 1000000 / 15; // 15 Hz
    
    uint8_t rx_buffer[32];
    uint8_t rx_count = 0;
    
    while (time_us_32() < end_time) {
        uint32_t now = time_us_32();
        
        // Send test byte at regular intervals
        if (now >= next_send_time) {
            mdat_send_byte(mdat, test_byte);
            next_send_time += interval;
        }
        
        // Check for received bytes
        uint64_t timestamp;
        size_t bytes_received = mdat_receive_bytes(mdat, rx_buffer, sizeof(rx_buffer), &timestamp);
        
        if (bytes_received > 0) {
            for (size_t i = 0; i < bytes_received; i++) {
                if (rx_buffer[i] == test_byte) {
                    rx_count++;
                }
            }
        }
        
        // Small delay to prevent tight loop
        sleep_ms(1);
    }
    
    printf("Test completed. Received %u valid test bytes\n", rx_count);
    
    // Consider test successful if we received at least some bytes
    return rx_count > 0;
}
