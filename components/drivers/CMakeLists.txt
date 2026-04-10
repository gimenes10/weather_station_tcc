idf_component_register(
    SRCS
        "i2c_bus.c"
        "adc_reader.c"
        "oled_ssd1306.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc
    PRIV_REQUIRES board_config
)
