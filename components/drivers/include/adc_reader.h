/**
 * @file adc_reader.h
 * @brief Leitura ADC oneshot para sensores analógicos (MQ-7, MQ-2, chuva).
 */
#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

/**
 * @brief Inicializa a unidade ADC1 e configura os três canais analógicos.
 * @param[out] handle  Handle da unidade ADC criada
 * @return ESP_OK em sucesso
 */
esp_err_t adc_reader_init(adc_oneshot_unit_handle_t *handle);

/**
 * @brief Lê o valor bruto de um canal ADC (0..4095 para 12 bits).
 * @param handle   Handle da unidade ADC
 * @param channel  Canal a ser lido
 * @param[out] raw Valor bruto lido
 * @return ESP_OK em sucesso
 */
esp_err_t adc_reader_read_raw(adc_oneshot_unit_handle_t handle,
                              adc_channel_t channel, int *raw);
