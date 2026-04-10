#include "rain_sensor.h"
#include "pin_config.h"
#include "adc_reader.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rain";

#define ADC_MAX_RAW     4095
#define ADC_REF_MV      3100.0f
#define RAIN_THRESHOLD  2000    /* Abaixo desse valor bruto = chovendo */
#define POWER_SETTLE_MS 100     /* Tempo para estabilizar após ligar VCC */

esp_err_t rain_sensor_init(void)
{
    esp_err_t ret;

    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << RAIN_POWER_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar GPIO de power: %s", esp_err_to_name(ret));
        goto err_gpio;
    }

    /* Sensor desligado por padrão */
    gpio_set_level(RAIN_POWER_PIN, 0);

    ESP_LOGI(TAG, "Sensor de chuva configurado (power=GPIO%d, adc=CH%d)",
             RAIN_POWER_PIN, RAIN_ADC_CHANNEL);
    return ESP_OK;

err_gpio:
    return ret;
}

esp_err_t rain_sensor_read(adc_oneshot_unit_handle_t adc, rain_data_t *data)
{
    esp_err_t ret;

    /* Liga alimentação do sensor */
    gpio_set_level(RAIN_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(POWER_SETTLE_MS));

    /* Lê ADC */
    ret = adc_reader_read_raw(adc, RAIN_ADC_CHANNEL, &data->raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura ADC: %s", esp_err_to_name(ret));
        goto err_read;
    }

    data->voltage    = ((float)data->raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;
    data->is_raining = (data->raw < RAIN_THRESHOLD);

    /* Desliga alimentação imediatamente */
    gpio_set_level(RAIN_POWER_PIN, 0);
    return ESP_OK;

err_read:
    gpio_set_level(RAIN_POWER_PIN, 0);
    return ret;
}
