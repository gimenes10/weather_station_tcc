idf_component_register(
    SRCS
        "i2c_bus.c"
        "adc_reader.c"
        "lora_sx1262.c"
        "reed_counter.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_spi esp_adc
    PRIV_REQUIRES board_config esp_timer
)
