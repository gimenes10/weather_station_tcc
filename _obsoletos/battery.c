#include "battery.h"
#include "adc_reader.h"
#include "pin_config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

/*
 * TODO conferir com multímetro: divisor 390k/100k da placa (×4,9). A
 * impedância alta faz o ADC ler um pouco abaixo (a Meshtastic corrige ×1,045).
 */
#define VBAT_SCALE      4.9f
#define SETTLE_MS       10
#define SAMPLES         16

static RTC_DATA_ATTR int s_active_level = -1;   /* -1 = revisão ainda não detectada */

static esp_err_t ctrl_idle(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << VBAT_CTRL_PIN,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

esp_err_t battery_init(bool cold_boot)
{
    esp_err_t ret = ctrl_idle();

    if (ret != ESP_OK || (!cold_boot && s_active_level >= 0)) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    /* Pull-up da placa (V3/V3.1) → nível alto em repouso → ativo em baixo */
    s_active_level = gpio_get_level(VBAT_CTRL_PIN) ? 0 : 1;
    ESP_LOGI(TAG, "Heltec %s: ADC_Ctrl ativo em nivel %s",
             s_active_level ? "V3.2" : "V3/V3.1", s_active_level ? "alto" : "baixo");
    return ESP_OK;
}

esp_err_t battery_read_mv(int *mv)
{
    int       pin_mv;
    esp_err_t ret = gpio_set_direction(VBAT_CTRL_PIN, GPIO_MODE_OUTPUT);

    if (ret != ESP_OK) {
        goto err;
    }
    gpio_set_level(VBAT_CTRL_PIN, s_active_level);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    ret = adc_reader_mv(VBAT_ADC_CHANNEL, SAMPLES, &pin_mv);
    if (ret == ESP_OK) {
        *mv = (int)((float)pin_mv * VBAT_SCALE);
    }

err:
    (void)ctrl_idle();   /* Volta ao repouso: divisor desligado */
    return ret;
}
