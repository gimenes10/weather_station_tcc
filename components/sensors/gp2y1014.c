#include "gp2y1014.h"
#include "adc_reader.h"
#include "pin_config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gp2y";

#define PULSE_US            320
#define SAMPLE_US           280
#define PERIOD_MS           10
#define PULSES              20
#define WARMUP_PULSES       2       /* Descartados: estabilização do LED e do capacitor */

#define DIV_NUM             32      /* (12k + 20k) / 20k */
#define DIV_DEN             20
#define K_MV_PER_UGM3       5.0f    /* 0,5 V / 100 µg/m³ */
#define VO_MIN_MV           100     /* Abaixo disso: sensor solto (o divisor puxa o pino a 0) */

/* Subida máxima do baseline: 10 mV/dia ≈ 20 µg/m³/dia de deriva absorvida */
#define VOC_CREEP_MV_PER_US (10.0f / 86400e6f)

static float   s_voc_mv;   /* 0 = ainda sem baseline */
static int64_t s_voc_us;   /* Instante da última atualização do baseline */

esp_err_t gp2y_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << GP2Y_LED_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   /* Pull-down externo na base de Q2 */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);

    if (ret == ESP_OK) {
        gpio_set_level(GP2Y_LED_PIN, 0);
    }
    return ret;
}

static void busy_until(int64_t t0, int64_t us)
{
    while (esp_timer_get_time() - t0 < us) {
    }
}

esp_err_t gp2y_read(gp2y_data_t *out)
{
    const UBaseType_t prio = uxTaskPriorityGet(NULL);
    TickType_t        last = xTaskGetTickCount();
    int32_t           sum  = 0;
    int               n    = 0;
    int               raw;
    esp_err_t         ret  = ESP_OK;

    /*
     * adc_oneshot_read não pode rodar em seção crítica: a janela de 280 µs
     * é garantida pela prioridade máxima (só ISRs interrompem, com jitter
     * de poucos µs). O LED fica aceso no máximo PULSE_US + tempo da leitura.
     */
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

    for (int i = 0; i < WARMUP_PULSES + PULSES; i++) {
        const int64_t t0 = esp_timer_get_time();

        gpio_set_level(GP2Y_LED_PIN, 1);
        busy_until(t0, SAMPLE_US);
        ret = adc_reader_raw(GP2Y_ADC_CHANNEL, &raw);
        busy_until(t0, PULSE_US);
        gpio_set_level(GP2Y_LED_PIN, 0);

        if (ret != ESP_OK) {
            goto err;
        }
        if (i >= WARMUP_PULSES) {
            sum += raw;
            n++;
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS));
    }
    vTaskPrioritySet(NULL, prio);

    out->vo_mv = adc_reader_to_mv(GP2Y_ADC_CHANNEL, (int)(sum / n)) * DIV_NUM / DIV_DEN;
    if (out->vo_mv < VO_MIN_MV) {
        ret = ESP_ERR_NOT_FOUND;
        goto err_vo;
    }

    /* Baseline: mínimo visto, com subida lenta para seguir a deriva */
    const int64_t now = esp_timer_get_time();

    if (s_voc_mv <= 0.0f) {
        s_voc_mv = (float)out->vo_mv;
    } else {
        s_voc_mv += (float)(now - s_voc_us) * VOC_CREEP_MV_PER_US;
        if ((float)out->vo_mv < s_voc_mv) {
            s_voc_mv = (float)out->vo_mv;
        }
    }
    s_voc_us = now;

    out->voc_mv    = (int)s_voc_mv;
    out->dust_ugm3 = ((float)out->vo_mv - s_voc_mv) / K_MV_PER_UGM3;
    if (out->dust_ugm3 < 0.0f) {
        out->dust_ugm3 = 0.0f;
    }
    return ESP_OK;

err:
    vTaskPrioritySet(NULL, prio);
    gpio_set_level(GP2Y_LED_PIN, 0);
err_vo:
    ESP_LOGW(TAG, "Sem leitura: %s", esp_err_to_name(ret));
    return ret;
}
