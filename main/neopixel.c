// neopixel.c

#include <stdio.h>
#include "driver/rmt.h"
#include "led_strip.h"
#include "led_strip_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NUM_LEDS 10  // We're using 10 LEDs


#define LED_STRIP_GPIO_PIN    18     // GPIO pin where the NeoPixel string is connected
#define LED_STRIP_NUM_LEDS    10     // Number of LEDs in your NeoPixel string

static led_strip_t *strip = NULL;

// Initialize the NeoPixel LED strip with the given GPIO pin and brightness
void neopixel_init(gpio_num_t gpio_pin, uint8_t brightness) {
    // Initialize the RMT TX channel
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio_pin, RMT_CHANNEL_0);
    config.clk_div = 2;  // RMT clock divider
    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL_0, 0, 0);

    // Configuration for the LED strip (NeoPixel string)
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // Pin connected to the NeoPixel string
        .max_leds = LED_STRIP_NUM_LEDS        // Number of LEDs in the NeoPixel string
    };

    // Install the LED strip driver for NeoPixels
//    strip = led_strip_new_rmt_ws2812(&strip_config);

//led_strip_new_rmt_device(strip_config, config, strip);


    // Initialize the string with zero brightness (off)
 //   led_strip_clear(strip);


    // Install the WS2812 driver (for NeoPixel)
 //   strip = led_strip_init(RMT_CHANNEL_0, NUM_LEDS);
 //   strip->clear(strip, 50);  // Clear all LEDs

    // Set the global brightness level for the LEDs
//    strip->set_brightness(strip, brightness);
}

// Set the color of a specific LED (by index) using RGB values
void neopixel_set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
//    if (strip != NULL && index >= 0 && index < NUM_LEDS) {
//        strip->set_pixel(strip, index, r, g, b);
 //       strip->refresh(strip, 100);  // Update the LED strip with new data
//    }
}
