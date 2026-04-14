/**
 * @file sensor_health.h
 * @brief Monitor de saúde dos sensores com reinicialização automática.
 *
 * Detecta sensores travados (valores repetidos ou zerados) e
 * tenta reinicializá-los automaticamente.
 */
#pragma once

#include "bme280.h"
#include "mpu6050.h"
#include "bh1750.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* Número de leituras idênticas consecutivas para considerar travado */
#define HEALTH_STALE_THRESHOLD  3

typedef struct {
    /* Contadores de leituras idênticas consecutivas */
    uint8_t bme280_stale_count;
    uint8_t mpu6050_stale_count;
    uint8_t bh1750_stale_count;

    /* Últimos valores para comparação */
    float last_bme_temp;
    float last_bme_hum;
    float last_bme_press;
    float last_mpu_ax;
    float last_mpu_ay;
    float last_mpu_az;
    float last_lux;

    /* Contadores de reinicializações */
    uint32_t bme280_resets;
    uint32_t mpu6050_resets;
    uint32_t bh1750_resets;
} sensor_health_t;

void sensor_health_init(sensor_health_t *health);

bool sensor_health_check_bme280(sensor_health_t *health,
                                 bme280_handle_t *sensor,
                                 const bme280_data_t *data,
                                 i2c_master_bus_handle_t bus);

bool sensor_health_check_mpu6050(sensor_health_t *health,
                                  mpu6050_handle_t *sensor,
                                  const mpu6050_data_t *data,
                                  i2c_master_bus_handle_t bus);

bool sensor_health_check_bh1750(sensor_health_t *health,
                                 bh1750_handle_t *sensor,
                                 float lux,
                                 i2c_master_bus_handle_t bus);