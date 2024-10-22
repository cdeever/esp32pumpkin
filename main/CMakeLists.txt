
# Register your main component
idf_component_register(SRCS "esp32pumpkin.c" "neopixel.c"  # Your main source file
        PRIV_REQUIRES esp_driver_i2s esp_driver_gpio driver
        INCLUDE_DIRS "."  # Adjust if you have specific include directories
)