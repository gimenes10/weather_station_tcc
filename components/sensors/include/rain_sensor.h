/**
 * @file rain_sensor.h
 * @brief FC-22 (comparador) + placa de chuva, lido pela saída analógica AO.
 *
 * O módulo fica alimentado o tempo todo pelo GPIO38 (nível alto).
 * Seco = AO alto; molhado = AO baixo.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    int  mv;        /* Tensão no AO */
    bool wet;       /* Abaixo do limiar */
} rain_data_t;

/** @brief Liga a alimentação do módulo (GPIO38). */
esp_err_t rain_sensor_init(void);

/** @brief Lê o AO (média de 4 amostras). */
esp_err_t rain_sensor_read(rain_data_t *out);
