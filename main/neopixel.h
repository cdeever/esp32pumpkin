// neopixel.h

#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <stdint.h>
#include "driver/gpio.h"

// Initialize the NeoPixel LED strip with a specific GPIO pin and brightness level
void neopixel_init(gpio_num_t gpio_pin, uint8_t brightness);

// Set the color of a specific LED
void neopixel_set_color(int index, uint8_t r, uint8_t g, uint8_t b);

#endif  // NEOPIXEL_H
