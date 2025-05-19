/**
 * @file tclk.h
 * @brief TCLK protocol implementation using Raspberry Pi Pico PIO
 */

#ifndef TCLK_H
#define TCLK_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

/**
 * @brief Structure representing a TCLK interface
 */
typedef struct {
    PIO pio;                // PIO instance
    uint sm_in;             // State machine for clockIN
    uint sm_decode;         // State machine for clockDecode
    uint sm_encode;         // State machine for clockEncode
    uint sm_out;            // State machine for clockOUT
    uint offset_in;         // Program offset for clockIN
    uint offset_decode;     // Program offset for clockDecode
    uint offset_encode;     // Program offset for clockEncode
    uint offset_out;        // Program offset for clockOUT
    uint clock_in_pin;      // GPIO pin for clock input
    uint clock_out_pin;     // GPIO pin for clock output
    uint dma_channel_in;    // DMA channel for input
    uint dma_channel_out;   // DMA channel for output
    bool is_running;        // Flag indicating if TCLK is running
} tclk_t;

/**
 * @brief Initialize the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @param pio_instance PIO instance to use (pio0 or pio1)
 * @param clock_in_pin GPIO pin for clock input
 * @param clock_out_pin GPIO pin for clock output
 * @return true if initialization was successful, false otherwise
 */
bool tclk_init(tclk_t* tclk, PIO pio_instance, uint clock_in_pin, uint clock_out_pin);

/**
 * @brief Start the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @return true if started successfully, false otherwise
 */
bool tclk_start(tclk_t* tclk);

/**
 * @brief Synchronize the TCLK interface with the incoming bit stream
 * 
 * This function attempts to find a valid TCLK pattern by shifting the bit stream
 * until a valid pattern is found. It should be called after tclk_start().
 * This function will wait indefinitely until a valid signal is detected,
 * even if the device is unplugged and later reconnected.
 * 
 * @param tclk Pointer to the TCLK structure
 * @return true if synchronized successfully, false otherwise (only if tclk is invalid)
 */
bool tclk_synchronize(tclk_t* tclk);

/**
 * @brief Stop the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 */
void tclk_stop(tclk_t* tclk);

/**
 * @brief Send a byte over the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @param byte Byte to send
 * @return true if sent successfully, false otherwise
 */
bool tclk_send_byte(tclk_t* tclk, uint8_t byte);

/**
 * @brief Send multiple bytes over the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @param buffer Buffer containing bytes to send
 * @param length Number of bytes to send
 * @return Number of bytes actually sent
 */
size_t tclk_send_bytes(tclk_t* tclk, const uint8_t* buffer, size_t length);

/**
 * @brief Receive a byte from the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @param byte Pointer to store the received byte
 * @return true if a byte was received successfully, false if no data available
 */
bool tclk_receive_byte(tclk_t* tclk, uint8_t* byte);

/**
 * @brief Receive multiple bytes from the TCLK interface
 * 
 * @param tclk Pointer to the TCLK structure
 * @param buffer Buffer to store the received bytes
 * @param max_length Maximum number of bytes to receive
 * @param last_bit_timestamp Pointer to store the timestamp of the last bit (can be NULL)
 * @return Number of bytes actually received
 */
size_t tclk_receive_bytes(tclk_t* tclk, uint8_t* buffer, size_t max_length, uint64_t* last_bit_timestamp);

/**
 * @brief Clean up the TCLK interface and free resources
 * 
 * @param tclk Pointer to the TCLK structure
 */
void tclk_deinit(tclk_t* tclk);

/**
 * @brief Run a test loop for TCLK with jumpered pins
 * 
 * This function creates a test loop that encodes a 15 Hz event (0x1F)
 * for 5 seconds and decodes the pin in real time. This is useful for
 * testing when the input and output pins are jumpered together.
 * 
 * @param tclk Pointer to the TCLK structure
 * @return true if test completed successfully, false otherwise
 */
bool tclk_run_test_loop(tclk_t* tclk);

#endif // TCLK_H
