/**
 * @file adc_reader.h
 * @brief ADC1 oneshot com calibração por curve fitting.
 *
 * Canais e atenuações:
 *   GPIO7 (CH6) Vo do GP2Y pelo divisor  12 dB
 *   GPIO2 (CH1) AO do FC-22              12 dB
 */
#pragma once

#include "hal/adc_types.h"
#include "esp_err.h"

esp_err_t adc_reader_init(void);

/** @brief Leitura bruta (0..4095). */
esp_err_t adc_reader_raw(adc_channel_t ch, int *raw);

/** @brief Converte uma leitura bruta do canal para mV no pino. */
int adc_reader_to_mv(adc_channel_t ch, int raw);

/** @brief Média de `n` leituras, em mV no pino. */
esp_err_t adc_reader_mv(adc_channel_t ch, int n, int *mv);
