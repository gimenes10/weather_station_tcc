#include "pulse.h"
#include "pin_config.h"
#include "ulp_pulse.h"          /* Gerado pelo build do ULP: símbolos ulp_* */
#include "ulp_riscv.h"
#include "ulp_common.h"
#include "driver/rtc_io.h"
#include "esp_rtc_time.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "pulse";

extern const uint8_t ulp_bin_start[] asm("_binary_ulp_pulse_bin_start");
extern const uint8_t ulp_bin_end[]   asm("_binary_ulp_pulse_bin_end");

/* ---------- Amostragem do ULP ---------- */
#define TICK_US             2000U       /* 500 Hz */
#define SLOT_US             1000000U    /* janela da rajada */
#define WIND_MIN_OPEN_US    4000U       /* 50 m/s ≈ 38 rot/s: ~20 ms aberto por volta */
#define RAIN_MIN_OPEN_US    50000U      /* 200 mm/h a 0,2 mm/basculada = 1 a cada 3,6 s */

/* ---------- Calibração (TODO) ---------- */
/*
 * Raio = eixo → centro da concha; K = v_vento / v_concha (conchas: ~2..3).
 * Depois da calibração em campo (regressão de rot/s × m/s), a inclinação
 * substitui WIND_MS_PER_RPS e o intercepto vira WIND_OFFSET_MS.
 */
#define WIND_CUP_RADIUS_M   0.070f      /* TODO: medir no modelo impresso */
#define WIND_CUP_FACTOR     3.0f        /* TODO: calibração */
#define WIND_OFFSET_MS      0.0f
#define WIND_MS_PER_RPS     (2.0f * 3.14159265f * WIND_CUP_RADIUS_M * WIND_CUP_FACTOR)

/* mm por basculada = V [mL] · 1000 / (π · (D_funil [mm] / 2)²) */
#define RAIN_MM_PER_TIP     0.2f        /* TODO: calibrar com volume conhecido */

#define MIN_INTERVAL_S      1.0f

/* Referência da leitura anterior (sobrevive ao deep sleep) */
static RTC_DATA_ATTR uint64_t s_prev_us;
static RTC_DATA_ATTR uint32_t s_prev_wind;
static RTC_DATA_ATTR uint32_t s_prev_ticks;

static esp_err_t ulp_start(void)
{
    esp_err_t ret;

    ulp_riscv_timer_stop();
    ulp_riscv_halt();

    ret = ulp_riscv_load_binary(ulp_bin_start, (size_t)(ulp_bin_end - ulp_bin_start));
    if (ret != ESP_OK) {
        goto err;
    }

    ulp_cfg_wind_gpio     = WIND_REED_PIN;
    ulp_cfg_rain_gpio     = GAUGE_REED_PIN;
    ulp_cfg_wind_min_open = (WIND_MIN_OPEN_US + TICK_US - 1U) / TICK_US;
    ulp_cfg_rain_min_open = (RAIN_MIN_OPEN_US + TICK_US - 1U) / TICK_US;
    ulp_cfg_slot_ticks    = SLOT_US / TICK_US;

    ret = ulp_set_wakeup_period(0, TICK_US);
    if (ret != ESP_OK) {
        goto err;
    }
    ret = ulp_riscv_run();
    if (ret != ESP_OK) {
        goto err;
    }

    s_prev_us    = esp_rtc_get_time_us();
    s_prev_wind  = 0;
    s_prev_ticks = 0;
    ESP_LOGI(TAG, "ULP iniciado (%u us por amostra)", TICK_US);
    return ESP_OK;

err:
    ESP_LOGE(TAG, "Falha ao iniciar o ULP: %s", esp_err_to_name(ret));
    return ret;
}

static esp_err_t reed_gpio_init(gpio_num_t io)
{
    esp_err_t ret = rtc_gpio_init(io);

    if (ret != ESP_OK) { goto err; }
    ret = rtc_gpio_set_direction(io, RTC_GPIO_MODE_INPUT_ONLY);
    if (ret != ESP_OK) { goto err; }
    ret = rtc_gpio_pullup_dis(io);          /* Pull-up externo de 100 kΩ */
    if (ret != ESP_OK) { goto err; }
    ret = rtc_gpio_pulldown_dis(io);
    if (ret != ESP_OK) { goto err; }
    ret = rtc_gpio_hold_en(io);             /* Trava a config do pad no deep sleep */

err:
    return ret;
}

esp_err_t pulse_init(bool cold_boot)
{
    esp_err_t ret;

    if (!cold_boot) {
        return ESP_OK;   /* ULP e RTC GPIO seguem configurados desde o boot a frio */
    }

    ret = reed_gpio_init(WIND_REED_PIN);
    if (ret != ESP_OK) { goto err; }
    ret = reed_gpio_init(GAUGE_REED_PIN);
    if (ret != ESP_OK) { goto err; }
    return ulp_start();

err:
    ESP_LOGE(TAG, "Falha nos RTC GPIO: %s", esp_err_to_name(ret));
    return ret;
}

static float rps_to_ms(float rps)
{
    return (rps > 0.0f) ? (WIND_MS_PER_RPS * rps + WIND_OFFSET_MS) : 0.0f;
}

esp_err_t pulse_read(pulse_data_t *out)
{
    const uint64_t now   = esp_rtc_get_time_us();
    const uint32_t wind  = ulp_wind_count;
    const uint32_t ticks = ulp_tick_count;
    const uint32_t gust  = ulp_gust_max;
    const float    dt_s  = (float)(now - s_prev_us) * 1e-6f;
    const uint32_t dtick = ticks - s_prev_ticks;

    out->rain_total_mm = (float)ulp_rain_count * RAIN_MM_PER_TIP;

    if (dt_s < MIN_INTERVAL_S) {
        return ESP_ERR_INVALID_STATE;   /* Primeiro ciclo após o boot a frio */
    }
    if (dtick == 0) {
        ESP_LOGE(TAG, "ULP parado: recarregando (acumulados zeram)");
        (void)ulp_start();
        return ESP_ERR_INVALID_STATE;
    }

    ulp_gust_reset = 1;   /* Nova janela de rajada a partir de agora */

    const float tick_s   = dt_s / (float)dtick;
    const float gust_s   = (float)(ulp_gust_slots * ulp_cfg_slot_ticks) * tick_s;
    const float avg_rps  = (float)(wind - s_prev_wind) / dt_s;
    const float gust_rps = (float)gust / gust_s;

    out->wind_avg_ms  = rps_to_ms(avg_rps);
    out->wind_gust_ms = rps_to_ms(gust_rps);
    if (out->wind_gust_ms < out->wind_avg_ms) {
        out->wind_gust_ms = out->wind_avg_ms;   /* Rajada nunca abaixo da média */
    }

    s_prev_us    = now;
    s_prev_wind  = wind;
    s_prev_ticks = ticks;
    return ESP_OK;
}
