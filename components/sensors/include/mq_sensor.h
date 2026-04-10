/**
 * @file mq_sensor.h
 * @brief Leitura dos sensores analógicos MQ-7 (CO) e MQ-2 (GLP/metano/H₂).
 *
 * Os valores retornados são brutos do ADC (0..4095).
 * A conversão para ppm requer calibração com gás de referência,
 * o que será implementado numa etapa posterior.
 */
#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

typedef struct {
    int mq7_raw;     /* ADC bruto do MQ-7 (CO)              */
    int mq2_raw;     /* ADC bruto do MQ-2 (GLP/metano)      */
    float mq7_voltage; /* Tensão no pino AOUT do MQ-7 (V)   */
    float mq2_voltage; /* Tensão no pino AOUT do MQ-2 (V)   */
} mq_data_t;

esp_err_t mq_sensor_read(adc_oneshot_unit_handle_t adc, mq_data_t *data);