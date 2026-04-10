idf_component_register(
    SRCS
        "bme280.c"
        "mpu6050.c"
        "bh1750.c"
        "mq_sensor.c"
        "rain_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc
    PRIV_REQUIRES board_config drivers
)
