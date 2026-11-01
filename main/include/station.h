/**
 * @file station.h
 * @brief Nó sensor: lê os sensores e transmite pelo LoRa a cada STATION_TX_PERIOD_S.
 */
#pragma once

#include "esp_err.h"

#define STATION_TX_PERIOD_S  60

/** @brief Inicializa barramentos, sensores e rádio. Falha só se o LoRa falhar. */
esp_err_t station_init(void);

/** @brief Loop de leitura e transmissão. Não retorna. */
void station_run(void) __attribute__((noreturn));
