#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#include "485802__grez1__scream14.h"
#include "132806__nanakisan__evil-laugh-12.h"

// PCM data from sample.h
extern const unsigned int sampleRate;
extern const unsigned int sampleCount;
extern const signed char samples[];  // Your PCM audio data

// I2S configuration
#define I2S_NUM         (0)  // I2S port number
#define I2S_BCK_IO      (37) // I2S Bit Clock
#define I2S_WS_IO       (33) // I2S Word Select (LR Clock)
#define I2S_DO_IO       (34) // I2S Data Out

void i2s_init() {
    // I2S driver configuration
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sampleRate,  // Set the sample rate (e.g., 44100 Hz)
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // Using 16-bit samples
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // Mono to both channels
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level
        .dma_buf_count = 8,   // Number of buffers
        .dma_buf_len = 1024,  // Size of each buffer
        .use_apll = false,
        .tx_desc_auto_clear = true,  // Auto clear the descriptor on underflow
    };

    // I2S pin configuration
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,   // Bit Clock
        .ws_io_num = I2S_WS_IO,     // Word Select (LR clock)
        .data_out_num = I2S_DO_IO,  // Data out
        .data_in_num = I2S_PIN_NO_CHANGE  // Not using data in
    };

    // Install and start I2S driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_set_clk(I2S_NUM, sampleRate, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void play_audio() {
    int i = 0;
    int16_t sample16;
    size_t bytes_written;

    while (i < sampleCount) {
        // Convert 8-bit signed sample to 16-bit signed (centered around 0)
        sample16 = (int16_t)samples[i] << 8;

        // Send the sample to the I2S peripheral
        i2s_write(I2S_NUM, &sample16, sizeof(sample16), &bytes_written, portMAX_DELAY);

        i++;
    }
}

void app_main() {
    // Initialize I2S
    i2s_init();

    // Start playing the audio
    play_audio();
}

