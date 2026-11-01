/**
 * @file gp2y1014.h
 * @brief Sensor óptico de poeira/fumaça Sharp GP2Y1014AU0F.
 *
 * O LED é acionado por Q2 (BC547): GPIO47 em nível ALTO acende. Cada pulso
 * dura 0,32 ms com amostra do Vo a 0,28 ms, um pulso a cada 10 ms (datasheet).
 * O Vo chega ao GPIO7 pelo divisor 12k/20k (×0,625).
 *
 *   densidade [µg/m³] = (Vo − Voc) / K,   K = 0,5 V por 100 µg/m³ (típico)
 *
 * Voc (saída sem poeira) varia de unidade para unidade (0–1,5 V, típ. 0,9 V).
 * O baseline acompanha o menor Vo visto e sobe devagar para seguir a deriva.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    int   vo_mv;        /* Vo médio no pino 5 do sensor */
    int   voc_mv;       /* Baseline usado no cálculo */
    float dust_ugm3;
} gp2y_data_t;

esp_err_t gp2y_init(void);

/**
 * @brief Faz a rajada de pulsos (~200 ms) e calcula a densidade.
 * @return ESP_ERR_NOT_FOUND se o Vo indicar sensor ausente
 */
esp_err_t gp2y_read(gp2y_data_t *out);
