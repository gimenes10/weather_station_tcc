#include "adc_reader.h"
#include "pin_config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "adc";

typedef struct {
    adc_channel_t     ch;
    adc_atten_t       atten;
    adc_cali_handle_t cali;     /* NULL = sem eFuse de calibração: conversão linear */
    int               full_mv;  /* Fundo de escala nominal da atenuação (fallback) */
} chan_t;

static adc_oneshot_unit_handle_t s_unit;
static chan_t s_ch[] = {
    { GP2Y_ADC_CHANNEL, ADC_ATTEN_DB_12,  NULL, 3100 },
    { RAIN_ADC_CHANNEL, ADC_ATTEN_DB_12,  NULL, 3100 },
};

static chan_t *find(adc_channel_t ch)
{
    for (size_t i = 0; i < sizeof(s_ch) / sizeof(s_ch[0]); i++) {
        if (s_ch[i].ch == ch) {
            return &s_ch[i];
        }
    }
    return NULL;
}

esp_err_t adc_reader_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_unit);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar ADC1: %s", esp_err_to_name(ret));
        goto err_unit;
    }

    for (size_t i = 0; i < sizeof(s_ch) / sizeof(s_ch[0]); i++) {
        const adc_oneshot_chan_cfg_t chan_cfg = {
            .atten    = s_ch[i].atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ret = adc_oneshot_config_channel(s_unit, s_ch[i].ch, &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha no canal %d: %s", s_ch[i].ch, esp_err_to_name(ret));
            goto err_chan;
        }

        const adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .chan     = s_ch[i].ch,
            .atten    = s_ch[i].atten,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_ch[i].cali) != ESP_OK) {
            ESP_LOGW(TAG, "Canal %d sem calibracao de fabrica: conversao linear", s_ch[i].ch);
            s_ch[i].cali = NULL;
        }
    }
    return ESP_OK;

err_chan:
    adc_oneshot_del_unit(s_unit);
    s_unit = NULL;
err_unit:
    return ret;
}

esp_err_t adc_reader_raw(adc_channel_t ch, int *raw)
{
    return adc_oneshot_read(s_unit, ch, raw);
}

int adc_reader_to_mv(adc_channel_t ch, int raw)
{
    const chan_t *c  = find(ch);
    int           mv = 0;

    if (c == NULL) {
        return 0;
    }
    if (c->cali == NULL || adc_cali_raw_to_voltage(c->cali, raw, &mv) != ESP_OK) {
        mv = raw * c->full_mv / 4095;
    }
    return mv;
}

esp_err_t adc_reader_mv(adc_channel_t ch, int n, int *mv)
{
    int32_t   sum = 0;
    int       raw;
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < n; i++) {
        ret = adc_oneshot_read(s_unit, ch, &raw);
        if (ret != ESP_OK) {
            goto err_read;
        }
        sum += raw;
    }
    *mv = adc_reader_to_mv(ch, (int)(sum / n));
    return ESP_OK;

err_read:
    return ret;
}
