#include "wind_sensor.h"
#include "reed_counter.h"
#include "pin_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "wind";

/* ---------- Calibração ---------- */

/*
 * TODO calibrar: valores teóricos até a calibração em campo.
 * Raio = eixo de rotação → centro da concha.
 * Fator do anemômetro K = v_vento / v_concha (conchas hemisféricas: ~2..3).
 */
#define WIND_CUP_RADIUS_M     0.070f   /* TODO: medir no modelo impresso */
#define WIND_CUP_FACTOR       3.0f     /* TODO: substituir pela calibração */
#define WIND_PULSES_PER_REV   1U       /* Confirmado: 1 interrupção por volta */
#define WIND_OFFSET_MS        0.0f     /* Velocidade de partida (atrito), da calibração */

#define WIND_MS_PER_RPS       (2.0f * 3.14159265f * WIND_CUP_RADIUS_M * WIND_CUP_FACTOR)

/* Debounce: 2 ms aberto. Com 0,07 m e K=3, 50 m/s ≈ 38 rot/s → ~20 ms aberto por volta. */
#define WIND_MIN_OPEN_US      2000U

/* Rajada WMO: média móvel de 3 s, amostrada a cada 1 s */
#define WIND_GUST_WINDOW_S    3U
#define WIND_TICK_US          1000000ULL

/* ---------- Estado ---------- */

static reed_counter_t     s_rc;
static esp_timer_handle_t s_tick;
static portMUX_TYPE       s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Usados só no callback do esp_timer */
static uint32_t s_win[WIND_GUST_WINDOW_S];
static uint8_t  s_win_idx;
static uint32_t s_tick_count;

/* Compartilhado callback ↔ wind_sensor_read (protegido por s_lock) */
static uint32_t s_gust_pulses;   /* Máximo de pulsos em 3 s desde a última leitura */

/* Usados só em wind_sensor_read */
static uint32_t s_read_count;
static int64_t  s_read_us;

static float rps_to_ms(float rps)
{
    return (rps > 0.0f) ? (WIND_MS_PER_RPS * rps + WIND_OFFSET_MS) : 0.0f;
}

static void wind_tick_cb(void *arg)
{
    (void)arg;

    const uint32_t now = reed_counter_get(&s_rc);
    uint32_t sum = 0;

    s_win[s_win_idx] = now - s_tick_count;   /* Aritmética modular cobre o wrap */
    s_tick_count     = now;
    s_win_idx        = (uint8_t)((s_win_idx + 1U) % WIND_GUST_WINDOW_S);

    for (uint32_t i = 0; i < WIND_GUST_WINDOW_S; i++) {
        sum += s_win[i];
    }

    taskENTER_CRITICAL(&s_lock);
    if (sum > s_gust_pulses) {
        s_gust_pulses = sum;
    }
    taskEXIT_CRITICAL(&s_lock);
}

esp_err_t wind_sensor_init(void)
{
    esp_err_t ret;

    ret = reed_counter_init(&s_rc, WIND_REED_PIN, WIND_MIN_OPEN_US);
    if (ret != ESP_OK) {
        goto err_reed;
    }

    const esp_timer_create_args_t tick_args = {
        .callback        = wind_tick_cb,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "wind_tick",
        .skip_unhandled_events = true,
    };
    ret = esp_timer_create(&tick_args, &s_tick);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar timer: %s", esp_err_to_name(ret));
        goto err_timer;
    }

    s_read_us    = esp_timer_get_time();
    s_read_count = 0;

    ret = esp_timer_start_periodic(s_tick, WIND_TICK_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar timer: %s", esp_err_to_name(ret));
        goto err_start;
    }

    ESP_LOGI(TAG, "Anemometro em GPIO%d (%.3f m/s por rot/s, %u pulso/rot)",
             WIND_REED_PIN, WIND_MS_PER_RPS, WIND_PULSES_PER_REV);
    return ESP_OK;

err_start:
    esp_timer_delete(s_tick);
err_timer:
    gpio_isr_handler_remove(WIND_REED_PIN);
    gpio_reset_pin(WIND_REED_PIN);
err_reed:
    return ret;
}

esp_err_t wind_sensor_read(wind_data_t *data)
{
    const int64_t  now_us = esp_timer_get_time();
    const uint32_t count  = reed_counter_get(&s_rc);
    const float    dt_s   = (float)(now_us - s_read_us) * 1e-6f;
    uint32_t       gust_pulses;

    if (dt_s <= 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(&s_lock);
    gust_pulses   = s_gust_pulses;
    s_gust_pulses = 0;
    taskEXIT_CRITICAL(&s_lock);

    const float avg_rps  = (float)(count - s_read_count) / (dt_s * WIND_PULSES_PER_REV);
    const float gust_rps = (float)gust_pulses / ((float)WIND_GUST_WINDOW_S * WIND_PULSES_PER_REV);

    data->avg_ms  = rps_to_ms(avg_rps);
    data->gust_ms = rps_to_ms(gust_rps);

    /* Intervalo < 3 s ou borda da janela: rajada nunca abaixo da média */
    if (data->gust_ms < data->avg_ms) {
        data->gust_ms = data->avg_ms;
    }

    s_read_count = count;
    s_read_us    = now_us;
    return ESP_OK;
}
