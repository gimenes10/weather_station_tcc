/**
 * @file rain_sensor.h
 * @brief Driver do sensor de chuva FC-22 com controle de alimentação via GPIO.
 *
 * A alimentação do sensor é controlada por um GPIO para evitar corrosão
 * eletrolítica nas trilhas de cobre — liga apenas no momento da leitura.
 */
#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

typedef struct {
    int raw;           /* ADC bruto (0..4095): alto = seco, baixo = molhado */
    float voltage;     /* Tensão no pino AO (V)                             */
    bool is_raining;   /* true se abaixo do limiar                          */
} rain_data_t;

/**
 * @brief Configura o GPIO de alimentação do sensor de chuva.
 */
esp_err_t rain_sensor_init(void);

/**
 * @brief Liga o sensor, lê o ADC, desliga o sensor.
 * @param adc    Handle da unidade ADC
 * @param[out] data  Dados lidos
 * @return ESP_OK em sucesso
 */
esp_err_t rain_sensor_read(adc_oneshot_unit_handle_t adc, rain_data_t *data);
