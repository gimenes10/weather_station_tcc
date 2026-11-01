/**
 * @file rain_gauge.h
 * @brief Pluviômetro basculante (gangorra) com reed switch + ímã.
 *
 * Cada basculada = RAIN_MM_PER_TIP mm de precipitação.
 *   mm_por_basculada = V_basculada [mm³] / A_funil [mm²]
 *                    = V [mL] · 1000 / (π · (D_funil [mm] / 2)²)
 *
 * O acumulado é monotônico desde o boot do nó: pacotes LoRa perdidos não
 * perdem chuva (o próximo pacote traz o total). Reboot do nó zera o total.
 */
#pragma once

#include "esp_err.h"

esp_err_t rain_gauge_init(void);

/** @brief Precipitação acumulada desde o boot (mm), calculada de basculadas × mm. */
float rain_gauge_total_mm(void);
