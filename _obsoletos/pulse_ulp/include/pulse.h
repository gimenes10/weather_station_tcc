/**
 * @file pulse.h
 * @brief Anemômetro e pluviômetro contados pelo ULP RISC-V durante o deep sleep.
 *
 * Anemômetro (1 pulso por volta):
 *   f = pulsos / Δt                      [rot/s]
 *   v = WIND_MS_PER_RPS · f + offset     [m/s], se f > 0; v = 0 se f = 0
 * Rajada: maior média móvel de 3 s no intervalo, mesma conversão.
 * Δt vem do contador RTC; o período real do ULP sai de Δt / execuções, então
 * a deriva do oscilador RC lento afeta média e rajada do mesmo jeito.
 *
 * Pluviômetro: acumulado = basculadas × RAIN_MM_PER_TIP desde que o ULP foi
 * carregado (boot a frio). Pacote perdido não perde chuva; reset zera.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float wind_avg_ms;
    float wind_gust_ms;
    float rain_total_mm;
} pulse_data_t;

/**
 * @brief No boot a frio configura os RTC GPIO, carrega e inicia o ULP.
 *        Num wake do deep sleep só confere se o ULP continua rodando.
 */
esp_err_t pulse_init(bool cold_boot);

/**
 * @brief Calcula vento médio, rajada e acumulado desde a última leitura.
 * @return ESP_ERR_INVALID_STATE no primeiro ciclo (intervalo curto demais)
 *         ou se o ULP parou (é recarregado e os contadores zeram).
 */
esp_err_t pulse_read(pulse_data_t *out);
