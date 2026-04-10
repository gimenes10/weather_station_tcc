#include "mq_sensor.h"
#include "pin_config.h"
#include "adc_reader.h"
#include "esp_log.h"

static const char *TAG = "mq_sensor";

/* ADC 12-bit com 12dB atten: range ~0..3.1V */
#define ADC_MAX_RAW    4095
#define ADC_REF_MV     3100.0f

/*
 * Compensação do divisor resistivo (10kΩ + 18kΩ) no AOUT dos módulos MQ.
 * O divisor reduz a tensão por fator 18k/(10k+18k) = 9/14.
 * Para obter a tensão real no AOUT, multiplicamos a leitura por 14/9.
 * Saída máxima com 5V: 5 × 9/14 = 3.21V (seguro para ESP32-S3).
 *
 *   AOUT (MQ, até 5V) ──[10kΩ]──┬── GPIO (ESP32, max 3.3V)
 *                                │
 *                              [18kΩ]
 *                                │
 *                               GND
 */
#define MQ_DIVIDER_FACTOR  1.556f   /* 14/9 = inverso de 9/14 */

esp_err_t mq_sensor_read(adc_oneshot_unit_handle_t adc, mq_data_t *data)
{
    esp_err_t ret;

    ret = adc_reader_read_raw(adc, MQ7_ADC_CHANNEL, &data->mq7_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler MQ-7: %s", esp_err_to_name(ret));
        goto err_read;
    }

    ret = adc_reader_read_raw(adc, MQ2_ADC_CHANNEL, &data->mq2_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler MQ-2: %s", esp_err_to_name(ret));
        goto err_read;
    }

    /* Tensão real no AOUT = tensão no ADC × fator do divisor */
    float adc_mq7_v = ((float)data->mq7_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;
    float adc_mq2_v = ((float)data->mq2_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;

    data->mq7_voltage = adc_mq7_v * MQ_DIVIDER_FACTOR;
    data->mq2_voltage = adc_mq2_v * MQ_DIVIDER_FACTOR;

    return ESP_OK;

err_read:
    return ret;
}