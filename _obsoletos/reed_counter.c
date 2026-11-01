#include "reed_counter.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "reed";

static void IRAM_ATTR reed_isr(void *arg)
{
    reed_counter_t *rc  = (reed_counter_t *)arg;
    const int64_t   now = esp_timer_get_time();

    /* Abertura (nível 1): marca o início do período aberto */
    if (gpio_get_level(rc->gpio)) {
        rc->t_open_us = now;
        return;
    }

    /* Fechamento (nível 0): só conta se esteve aberto tempo suficiente */
    if ((now - rc->t_open_us) >= (int64_t)rc->min_open_us) {
        rc->count++;
    }
}

esp_err_t reed_counter_init(reed_counter_t *rc, gpio_num_t gpio, uint32_t min_open_us)
{
    esp_err_t ret;

    rc->gpio        = gpio;
    rc->min_open_us = min_open_us;
    rc->t_open_us   = 0;
    rc->count       = 0;

    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << gpio,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    /* Pull-up interno (~45 kΩ) */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: falha no gpio_config: %s", gpio, esp_err_to_name(ret));
        goto err_cfg;
    }

    /* ESP_ERR_INVALID_STATE = serviço já instalado por outra instância */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Falha ao instalar ISR service: %s", esp_err_to_name(ret));
        goto err_cfg;
    }

    ret = gpio_isr_handler_add(gpio, reed_isr, rc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: falha ao registrar ISR: %s", gpio, esp_err_to_name(ret));
        goto err_cfg;
    }

    ESP_LOGI(TAG, "Reed em GPIO%d (debounce %lu us aberto)",
             gpio, (unsigned long)min_open_us);
    return ESP_OK;

err_cfg:
    gpio_reset_pin(gpio);
    return ret;
}
