/**
 * @file mdat.h
 * @brief MDAT protocol implementation using Raspberry Pi Pico PIO
 */

#ifndef MDAT_H
#define MDAT_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

/**
 * @brief Structure representing a MDAT interface
 */
typedef struct {
    PIO pio;                // PIO instance
    uint sm_in;             // State machine for mdatIN
    uint sm_decode;         // State machine for mdatDecode
    uint sm_encode;         // State machine for mdatEncode
    uint sm_out;            // State machine for mdatOUT
    uint offset_in;         // Program offset for mdatIN
    uint offset_decode;     // Program offset for mdatDecode
    uint offset_encode;     // Program offset for mdatEncode
    uint offset_out;        // Program offset for mdatOUT
    uint clock_in_pin;      // GPIO pin for clock input
    uint clock_out_pin;     // GPIO pin for clock output
    uint dma_channel_in;    // DMA channel for input
    uint dma_channel_out;   // DMA channel for output
    bool is_running;        // Flag indicating if MDAT is running
} mdat_t;

/**
 * @brief Initialize the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @param pio_instance PIO instance to use (pio0 or pio1)
 * @param clock_in_pin GPIO pin for clock input
 * @param clock_out_pin GPIO pin for clock output
 * @return true if initialization was successful, false otherwise
 */
bool mdat_init(mdat_t* mdat, PIO pio_instance, uint clock_in_pin, uint clock_out_pin);

/**
 * @brief Start the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @return true if started successfully, false otherwise
 */
bool mdat_start(mdat_t* mdat);

/**
 * @brief Synchronize the MDAT interface with the incoming bit stream
 * 
 * This function attempts to find a valid MDAT pattern by shifting the bit stream
 * until a valid pattern is found. It should be called after mdat_start().
 * This function will wait indefinitely until a valid signal is detected,
 * even if the device is unplugged and later reconnected.
 * 
 * @param mdat Pointer to the MDAT structure
 * @return true if synchronized successfully, false otherwise (only if mdat is invalid)
 */
bool mdat_synchronize(mdat_t* mdat);

/**
 * @brief Stop the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 */
void mdat_stop(mdat_t* mdat);

/**
 * @brief Send a byte over the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @param byte Byte to send
 * @return true if sent successfully, false otherwise
 */
bool mdat_send_byte(mdat_t* mdat, uint8_t byte);

/**
 * @brief Send multiple bytes over the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @param buffer Buffer containing bytes to send
 * @param length Number of bytes to send
 * @return Number of bytes actually sent
 */
size_t mdat_send_bytes(mdat_t* mdat, const uint8_t* buffer, size_t length);

/**
 * @brief Receive a byte from the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @param byte Pointer to store the received byte
 * @return true if a byte was received successfully, false if no data available
 */
bool mdat_receive_byte(mdat_t* mdat, uint8_t* byte);

/**
 * @brief Receive multiple bytes from the MDAT interface
 * 
 * @param mdat Pointer to the MDAT structure
 * @param buffer Buffer to store the received bytes
 * @param max_length Maximum number of bytes to receive
 * @param last_bit_timestamp Pointer to store the timestamp of the last bit (can be NULL)
 * @return Number of bytes actually received
 */
size_t mdat_receive_bytes(mdat_t* mdat, uint8_t* buffer, size_t max_length, uint64_t* last_bit_timestamp);

/**
 * @brief Clean up the MDAT interface and free resources
 * 
 * @param mdat Pointer to the MDAT structure
 */
void mdat_deinit(mdat_t* mdat);

/**
 * @brief Run a test loop for MDAT with jumpered pins
 * 
 * This function creates a test loop that encodes a 15 Hz event (0x1F)
 * for 5 seconds and decodes the pin in real time. This is useful for
 * testing when the input and output pins are jumpered together.
 * 
 * @param mdat Pointer to the MDAT structure
 * @return true if test completed successfully, false otherwise
 */
bool mdat_run_test_loop(mdat_t* mdat);

#endif // MDAT_H
