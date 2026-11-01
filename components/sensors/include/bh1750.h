/**
 * @file bh1750.h
 * @brief BH1750 (GY-302) em one-time H-resolution com MTreg = 31.
 *
 * MTreg 31 (padrão 69) amplia o fundo de escala de ~54,6 klx para
 * ~121,6 klx, acima do sol direto. Resolução cai para ~1,85 lx.
 *   lux = contagem / 1,2 × 69 / MTreg / transmitância da cúpula
 * Após a medição o sensor volta sozinho ao power-down.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/* Pior caso do datasheet (180 ms em MTreg 69) escalado para MTreg 31 */
#define BH1750_MEAS_MS  81

esp_err_t bh1750_init(i2c_master_bus_handle_t bus);

/** @brief Liga, ajusta MTreg e dispara uma medição one-time. */
esp_err_t bh1750_trigger(void);

/** @brief Lê o resultado (chamar ≥ BH1750_MEAS_MS após o trigger). */
esp_err_t bh1750_read(float *lux);
