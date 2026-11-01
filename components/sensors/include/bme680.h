/**
 * @file bme680.h
 * @brief BME680/BME688 em modo forced: temperatura, umidade, pressão e
 *        resistência do gás (sem BSEC).
 *
 * Uma medição por ciclo: bme680_trigger() configura e dispara a medição
 * (aquecedor a 320 °C por 150 ms); bme680_read() espera o fim e compensa
 * com as fórmulas de ponto flutuante da BME68x API da Bosch.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    float gas_ohm;
    bool  gas_valid;    /* false: aquecedor não estabilizou ou medição de gás inválida */
} bme680_data_t;

/** @brief Adiciona ao barramento, confere o chip ID e carrega a calibração. */
esp_err_t bme680_init(i2c_master_bus_handle_t bus);

/**
 * @brief Configura oversampling e aquecedor e dispara uma medição forced.
 * @param[out] wait_ms Tempo mínimo até o resultado ficar pronto
 */
esp_err_t bme680_trigger(uint32_t *wait_ms);

/** @brief Espera o new_data e compensa a medição disparada. */
esp_err_t bme680_read(bme680_data_t *out);
