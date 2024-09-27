idf_component_register(SRCS "esp32pumpkin.c"
        PRIV_REQUIRES esp_driver_i2s esp_driver_gpio driver
	INCLUDE_DIRS )
