/**
 * @file weather_task.h
 * @brief Task principal que lê todos os sensores e atualiza OLED + serial.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Inicializa todos os periféricos e sensores.
 * @return ESP_OK se todos os sensores foram inicializados com sucesso
 */
esp_err_t weather_station_init(void);

/**
 * @brief Task FreeRTOS que faz leituras periódicas.
 * @param arg  Não utilizado
 */
void weather_task(void *arg);
