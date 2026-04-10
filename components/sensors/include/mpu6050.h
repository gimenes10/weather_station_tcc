/**
 * @file mpu6050.h
 * @brief Driver do sensor MPU-6050 — acelerômetro e giroscópio 6 DoF.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    float accel_x, accel_y, accel_z;  /* Aceleração em g       */
    float gyro_x, gyro_y, gyro_z;    /* Velocidade angular °/s */
    float temperature_c;              /* Temperatura interna °C */
} mpu6050_data_t;

typedef struct {
    i2c_master_dev_handle_t dev;
} mpu6050_handle_t;

esp_err_t mpu6050_init(mpu6050_handle_t *sensor, i2c_master_bus_handle_t bus);
esp_err_t mpu6050_read(mpu6050_handle_t *sensor, mpu6050_data_t *data);
