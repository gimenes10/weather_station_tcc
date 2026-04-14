idf_component_register(
    SRCS
        "bme280.c"
        "mpu6050.c"
        "bh1750.c"
        "mq_sensor.c"
        "rain_sensor.c"
        "sensor_health.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio esp_adc
    PRIV_REQUIRES board_config drivers
)