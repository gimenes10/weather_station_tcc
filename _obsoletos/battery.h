/**
 * @file battery.h
 * @brief Tensão da bateria pelo divisor da própria placa Heltec (GPIO1, GPIO37).
 *
 * Usado quando o INA219 #2 não responde. O GPIO37 liga o divisor:
 * V3/V3.1 têm pull-up nesse pino e o controle é ativo em nível baixo; a V3.2
 * troca por um transistor ativo em nível alto. A revisão é detectada no
 * primeiro boot lendo o nível do pino em repouso.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

/** @param cold_boot true no primeiro boot (detecta a revisão da placa) */
esp_err_t battery_init(bool cold_boot);

esp_err_t battery_read_mv(int *mv);
