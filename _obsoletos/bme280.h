/**
 * @file bme280.h
 * @brief Driver do sensor BME280 — temperatura, umidade e pressão atmosférica.
 *
 * Implementa leitura via I2C com compensação por coeficientes de calibração
 * conforme datasheet Bosch BST-BME280-DS002.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    float temperature_c;    /* Temperatura em °C  */
    float humidity_pct;     /* Umidade relativa %  */
    float pressure_hpa;     /* Pressão em hPa      */
} bme280_data_t;

typedef struct {
    i2c_master_dev_handle_t dev;

    /* Coeficientes de calibração (lidos uma vez no init) */
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3; int16_t dig_H2, dig_H4, dig_H5; int8_t dig_H6;
    int32_t  t_fine; /* Usado internamente na compensação */
} bme280_handle_t;

esp_err_t bme280_init(bme280_handle_t *sensor, i2c_master_bus_handle_t bus);
esp_err_t bme280_read(bme280_handle_t *sensor, bme280_data_t *data);
