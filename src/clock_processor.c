/**
 * @file clock_processor.c
 * @brief Implementation of clock signal processing functionality
 */

#include "clock_processor.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// Include the PIO program headers (will be generated during build)
#include "clockIN.pio.h"
#include "clockDecode.pio.h"

bool clock_processor_init(clock_processor_t* processor, PIO pio_instance, uint clock_pin) {
    if (!processor) return false;
    
    // Initialize the processor structure
    processor->pio = pio_instance;
    processor->clock_pin = clock_pin;
    processor->is_running = false;
    
    // Claim state machines
    processor->sm_in = pio_claim_unused_sm(processor->pio, true);
    if (processor->sm_in == -1) return false;
    
    processor->sm_decode = pio_claim_unused_sm(processor->pio, true);
    if (processor->sm_decode == -1) {
        pio_sm_unclaim(processor->pio, processor->sm_in);
        return false;
    }
    
    // Claim DMA channel
    processor->dma_channel = dma_claim_unused_channel(true);
    if (processor->dma_channel == -1) {
        pio_sm_unclaim(processor->pio, processor->sm_in);
        pio_sm_unclaim(processor->pio, processor->sm_decode);
        return false;
    }
    
    // Load PIO programs
    processor->offset_in = pio_add_program(processor->pio, &clockIN_program);
    processor->offset_decode = pio_add_program(processor->pio, &clockDecode_program);
    
    // Initialize the clockIN program
    clockIN_program_init(processor->pio, processor->sm_in, processor->offset_in, processor->clock_pin);
    
    // Initialize the clockDecode program
    clockDecode_program_init(processor->pio, processor->sm_decode, processor->offset_decode);
    
    return true;
}

bool clock_processor_start(clock_processor_t* processor) {
    if (!processor || processor->is_running) return false;
    
    // Enable state machines
    pio_sm_set_enabled(processor->pio, processor->sm_in, true);
    pio_sm_set_enabled(processor->pio, processor->sm_decode, true);
    
    // Set up DMA to transfer data from clockIN to clockDecode
    dma_channel_config c = dma_channel_get_default_config(processor->dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(processor->pio, processor->sm_in, false));
    
    dma_channel_configure(
        processor->dma_channel,
        &c,
        &processor->pio->txf[processor->sm_decode],  // Write to clockDecode TX FIFO
        &processor->pio->rxf[processor->sm_in],      // Read from clockIN RX FIFO
        0,                                           // Transfer forever
        true                                         // Start immediately
    );
    
    processor->is_running = true;
    return true;
}

void clock_processor_stop(clock_processor_t* processor) {
    if (!processor || !processor->is_running) return;
    
    // Disable state machines
    pio_sm_set_enabled(processor->pio, processor->sm_in, false);
    pio_sm_set_enabled(processor->pio, processor->sm_decode, false);
    
    // Abort DMA transfer
    dma_channel_abort(processor->dma_channel);
    
    processor->is_running = false;
}

bool clock_processor_read_bit(clock_processor_t* processor, bool* bit) {
    if (!processor || !processor->is_running || !bit) return false;
    
    // Check if there's data available
    if (pio_sm_is_rx_fifo_empty(processor->pio, processor->sm_decode)) {
        return false;
    }
    
    // Read a bit from the clockDecode state machine
    *bit = clockDecode_get_bit(processor->pio, processor->sm_decode);
    return true;
}

size_t clock_processor_read_bits(clock_processor_t* processor, uint8_t* buffer, size_t max_bits) {
    if (!processor || !processor->is_running || !buffer || max_bits == 0) return 0;
    
    size_t bits_read = 0;
    uint8_t current_byte = 0;
    uint8_t bit_position = 0;
    
    while (bits_read < max_bits) {
        bool bit;
        if (!clock_processor_read_bit(processor, &bit)) {
            // No more data available
            break;
        }
        
        // Pack bits into bytes
        if (bit) {
            current_byte |= (1 << bit_position);
        }
        
        bit_position++;
        bits_read++;
        
        if (bit_position == 8) {
            // We've filled a byte, store it and reset
            *buffer++ = current_byte;
            current_byte = 0;
            bit_position = 0;
        }
    }
    
    // If we have a partial byte at the end, store it too
    if (bit_position > 0) {
        *buffer = current_byte;
    }
    
    return bits_read;
}

void clock_processor_deinit(clock_processor_t* processor) {
    if (!processor) return;
    
    // Stop processing if running
    if (processor->is_running) {
        clock_processor_stop(processor);
    }
    
    // Free resources
    pio_sm_unclaim(processor->pio, processor->sm_in);
    pio_sm_unclaim(processor->pio, processor->sm_decode);
    dma_channel_unclaim(processor->dma_channel);
    
    // Clear the structure
    processor->pio = NULL;
    processor->sm_in = 0;
    processor->sm_decode = 0;
    processor->offset_in = 0;
    processor->offset_decode = 0;
    processor->clock_pin = 0;
    processor->dma_channel = 0;
    processor->is_running = false;
}
