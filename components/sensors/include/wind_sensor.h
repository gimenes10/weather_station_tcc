/**
 * @file wind_sensor.h
 * @brief Anemômetro de conchas com reed switch (contagem de pulsos por interrupção).
 *
 * Velocidade média: pulsos no intervalo entre duas leituras / tempo medido.
 * Rajada: maior média móvel de 3 s dentro do intervalo (definição WMO),
 * amostrada a cada 1 s por um esp_timer.
 *
 * Conversão (curva de calibração linear do anemômetro):
 *   f   = pulsos / (Δt · WIND_PULSES_PER_REV)          [rot/s]
 *   v   = WIND_MS_PER_RPS · f + WIND_OFFSET_MS          [m/s], se f > 0
 *   v   = 0                                             se f = 0
 */
#pragma once

#include "esp_err.h"

typedef struct {
    float avg_ms;   /* Velocidade média desde a última leitura (m/s) */
    float gust_ms;  /* Rajada: máxima média de 3 s no mesmo intervalo (m/s) */
} wind_data_t;

esp_err_t wind_sensor_init(void);

/**
 * @brief Calcula média e rajada desde a chamada anterior e reinicia a janela.
 *        A primeira chamada cobre o intervalo desde o init.
 */
esp_err_t wind_sensor_read(wind_data_t *data);
