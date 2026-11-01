#include "rain_gauge.h"
#include "reed_counter.h"
#include "pin_config.h"
#include "esp_log.h"

static const char *TAG = "gauge";

/* TODO calibrar: despejar volume conhecido devagar e contar basculadas */
#define RAIN_MM_PER_TIP      0.2f

/*
 * Debounce: 50 ms aberto. Chuva de 200 mm/h com 0,2 mm/basculada
 * = 1000 basculadas/h → uma a cada 3,6 s, muito acima do debounce.
 */
#define RAIN_MIN_OPEN_US     50000U

static reed_counter_t s_rc;

esp_err_t rain_gauge_init(void)
{
    esp_err_t ret = reed_counter_init(&s_rc, GAUGE_REED_PIN, RAIN_MIN_OPEN_US);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Pluviometro em GPIO%d (%.3f mm/basculada)",
                 GAUGE_REED_PIN, RAIN_MM_PER_TIP);
    }
    return ret;
}

float rain_gauge_total_mm(void)
{
    /* Recalculado do contador a cada leitura: sem acúmulo de erro de float */
    return (float)reed_counter_get(&s_rc) * RAIN_MM_PER_TIP;
}
