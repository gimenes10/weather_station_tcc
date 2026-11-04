/**
 * @file gp2y1014.h
 * @brief Sensor óptico de poeira/fumaça Sharp GP2Y1014AU0F.
 *
 * O LED é acionado por Q2 (BC547): GPIO6 em nível ALTO acende. Cada pulso
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
    float dust_ugm3;    /* Limitado a 500 quando saturado */
    bool  saturated;    /* Fora da faixa linear: o valor real é maior que dust_ugm3 */
} gp2y_data_t;

esp_err_t gp2y_init(void);

/**
 * @brief Faz a rajada de pulsos (~200 ms) e calcula a densidade.
 * @return ESP_ERR_NOT_FOUND se o Vo indicar sensor ausente
 */
esp_err_t gp2y_read(gp2y_data_t *out);

/*
 * Teste do driver do LED (GPIO6 → R3 → Q2). Com 1, o nó fica preso num laço
 * que alterna o GPIO6 a cada 5 s, para medir o pino LED do J9 com o multímetro.
 * Desconecte o GP2Y antes: com o Q2 conduzindo 5 s, o LED ficaria aceso direto.
 */
#define GP2Y_Q2_TEST  0

/** @brief Laço infinito do teste do Q2 (não retorna). */
void gp2y_q2_test(void);
