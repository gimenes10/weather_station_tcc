#include "adc_reader.h"
#include "pin_config.h"
#include "esp_log.h"

static const char *TAG = "adc";

esp_err_t adc_reader_init(adc_oneshot_unit_handle_t *handle)
{
    esp_err_t ret;

    /* Cria unidade ADC1 */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    ret = adc_oneshot_new_unit(&unit_cfg, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar unidade ADC1: %s", esp_err_to_name(ret));
        goto err_unit;
    }

    /* Configuração comum: 12 dB de atenuação -> leitura até ~3.3V */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    /* Configura os 3 canais */
    adc_channel_t channels[] = { MQ7_ADC_CHANNEL, MQ2_ADC_CHANNEL, RAIN_ADC_CHANNEL };
    for (int i = 0; i < 3; i++) {
        ret = adc_oneshot_config_channel(*handle, channels[i], &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao configurar canal %d: %s", channels[i], esp_err_to_name(ret));
            goto err_chan;
        }
    }

    ESP_LOGI(TAG, "ADC1 inicializado (3 canais, 12dB atten)");
    return ESP_OK;

err_chan:
    adc_oneshot_del_unit(*handle);
err_unit:
    return ret;
}

esp_err_t adc_reader_read_raw(adc_oneshot_unit_handle_t handle,
                              adc_channel_t channel, int *raw)
{
    return adc_oneshot_read(handle, channel, raw);
}
