/**
 * @file test_led_serial.cpp
 * @brief Minimal test program for LED and serial functionality on Raspberry Pi Pico
 */

#include "pico/stdlib.h"
#include <stdio.h>

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    while (true) {
        printf("Blink!\n");
        gpio_put(25, 1);
        sleep_ms(250);
        gpio_put(25, 0);
        sleep_ms(250);
    }
}
