#include "mq_sensor.h"
#include "pin_config.h"
#include "adc_reader.h"
#include "esp_log.h"

static const char *TAG = "mq_sensor";

/* ADC 12-bit com 12dB atten: range ~0..3.1V */
#define ADC_MAX_RAW    4095
#define ADC_REF_MV     3100.0f

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

    data->mq7_voltage = ((float)data->mq7_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;
    data->mq2_voltage = ((float)data->mq2_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;

    return ESP_OK;

err_read:
    return ret;
}
