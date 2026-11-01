idf_component_register(
    SRCS
        "bme680.c"
        "bh1750.c"
        "gp2y1014.c"
        "rain_sensor.c"
        "wind_sensor.c"
        "rain_gauge.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES board_config drivers esp_driver_gpio esp_timer
)
