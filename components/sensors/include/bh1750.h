/**
 * @file bh1750.h
 * @brief Driver do sensor BH1750 — luminosidade ambiente (lux).
 *
 * Utiliza modo One-Time H-Resolution para baixo consumo:
 * faz uma medição e entra em power-down automaticamente.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    i2c_master_dev_handle_t dev;
} bh1750_handle_t;

esp_err_t bh1750_init(bh1750_handle_t *sensor, i2c_master_bus_handle_t bus);

/**
 * @brief Faz uma leitura one-time de luminosidade.
 * @param sensor  Handle do sensor
 * @param[out] lux  Valor de luminosidade em lux
 * @return ESP_OK em sucesso
 */
esp_err_t bh1750_read(bh1750_handle_t *sensor, float *lux);
