#include "rain_sensor.h"
#include "adc_reader.h"
#include "pin_config.h"
#include "driver/gpio.h"

/* Limiar herdado do firmware anterior (raw 2000 com conversão linear) */
#define RAIN_WET_MV  1500

esp_err_t rain_sensor_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << RAIN_POWER_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);

    if (ret == ESP_OK) {
        ret = gpio_set_level(RAIN_POWER_PIN, 1);
    }
    return ret;
}

esp_err_t rain_sensor_read(rain_data_t *out)
{
    esp_err_t ret = adc_reader_mv(RAIN_ADC_CHANNEL, 4, &out->mv);

    if (ret == ESP_OK) {
        out->wet = (out->mv < RAIN_WET_MV);
    }
    return ret;
}
