// neopixel.c

#include <stdio.h>
#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "neopixel.h"

#define NUM_LEDS 10  // We're using 10 LEDs

#define LED_STRIP_GPIO_PIN    18     // GPIO pin where the NeoPixel string is connected
#define LED_STRIP_NUM_LEDS    10     // Number of LEDs in your NeoPixel string

static led_strip_t *strip = NULL;

// Initialize the NeoPixel LED strip with the given GPIO pin and brightness
led_strip_handle_t neopixel_init() {
//gpio_num_t gpio_pin, uint8_t brightness

led_strip_handle_t led_strip;

/* LED strip initialization with the GPIO and pixels number*/
led_strip_config_t strip_config = {
    .strip_gpio_num = 39, // The GPIO that connected to the LED strip's data line
    .max_leds = 10, // The number of LEDs in the strip,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
    .led_model = LED_MODEL_WS2812, // LED strip model
    .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
};

led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
    .flags.with_dma = false, // whether to enable the DMA feature
};
ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

  return led_strip;

}


void show_lights(led_strip_handle_t led_strip) {
 
    int blink_duration = 50;  // Blink on/off every 200 milliseconds (adjust for speed)
    int total_blinks = 3000 / (2 * blink_duration);  // Total number of blinks in 3 seconds (on + off counts as one blink)

    for (int i = 0; i < total_blinks; i++) {
        // Turn the LEDs on with low brightness (white color)
        for (int j = 0; j < 10; j++) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, j, 5, 5, 5));  // Low brightness white
        }
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));  // Send data to the LED strip
 
        vTaskDelay(pdMS_TO_TICKS(blink_duration));  // Wait for blink_duration (200 ms)

        // Turn the LEDs off
        ESP_ERROR_CHECK(led_strip_clear(led_strip));  // Turn off all LEDs
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));  // Refresh the strip
 
        vTaskDelay(pdMS_TO_TICKS(blink_duration));  // Wait for blink_duration (200 ms)
    }
}