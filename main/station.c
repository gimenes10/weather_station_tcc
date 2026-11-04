#include "station.h"
#include "pin_config.h"
#include "i2c_bus.h"
#include "adc_reader.h"
#include "lora_sx1262.h"
#include "lora_packet.h"
#include "bme680.h"
#include "bh1750.h"
#include "gp2y1014.h"
#include "rain_sensor.h"
#include "wind_sensor.h"
#include "rain_gauge.h"

#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "station";

/* Recuperação do barramento I2C: ciclos seguidos sem BME680 nem BH1750 */
#define I2C_FAIL_RESET      3       /* → i2c_bus_reset() */
#define I2C_FAIL_RESTART    10      /* → esp_restart() (~10 min) */

typedef struct {
    esp_err_t     env_err;
    bme680_data_t env;
    esp_err_t     lux_err;
    float         lux;
    esp_err_t     dust_err;
    gp2y_data_t   dust;
    esp_err_t     rain_err;
    rain_data_t   rain;
    esp_err_t     wind_err;
    wind_data_t   wind;
    float         rain_total_mm;
} readings_t;

static i2c_master_bus_handle_t s_bus;
static lora_sx1262_handle_t    s_lora;
static bool                    s_adc_ok;
static bool                    s_gp2y_ok;
static bool                    s_rain_ok;
static bool                    s_wind_ok;
static bool                    s_gauge_ok;
static uint16_t                s_seq;
static uint32_t                s_i2c_fail;
static bool                    s_i2c_missing = true;   /* Algum sensor I2C falhou no último ciclo */

/* ---------- Inicialização ---------- */

esp_err_t station_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Estacao meteorologica | Heltec V3 | pacote v%d (%u bytes) a cada %d s",
             LORA_PKT_VERSION, (unsigned)sizeof(lora_packet_t), STATION_TX_PERIOD_S);

    ret = i2c_bus_init(&s_bus);
    if (ret != ESP_OK) {
        goto err;
    }
    (void)i2c_bus_scan(s_bus);

    /* Sensores I2C ausentes no boot são tentados de novo a cada ciclo */
    (void)bme680_init(s_bus);
    (void)bh1750_init(s_bus);

    s_adc_ok   = (adc_reader_init() == ESP_OK);
    s_gp2y_ok  = s_adc_ok && (gp2y_init() == ESP_OK);
#if GP2Y_Q2_TEST
    gp2y_q2_test();   /* Não retorna: só para medir o driver do LED */
#endif
    s_rain_ok  = s_adc_ok && (rain_sensor_init() == ESP_OK);
    s_wind_ok  = (wind_sensor_init() == ESP_OK);
    s_gauge_ok = (rain_gauge_init() == ESP_OK);

    /* Sem rádio o nó não tem função */
    ret = lora_sx1262_init(&s_lora);
    if (ret != ESP_OK) {
        goto err;
    }
    return ESP_OK;

err:
    return ret;
}

/* ---------- Medição ---------- */

/** Dispara a medição; se o sensor ainda não respondeu desde o boot, tenta inicializá-lo. */
static esp_err_t bme_start(uint32_t *wait_ms)
{
    esp_err_t ret = bme680_trigger(wait_ms);

    if (ret == ESP_ERR_INVALID_STATE && bme680_init(s_bus) == ESP_OK) {
        ret = bme680_trigger(wait_ms);
    }
    return ret;
}

static esp_err_t bh_start(void)
{
    esp_err_t ret = bh1750_trigger();

    if (ret == ESP_ERR_INVALID_STATE && bh1750_init(s_bus) == ESP_OK) {
        ret = bh1750_trigger();
    }
    return ret;
}

static void measure(readings_t *r)
{
    const TickType_t t0      = xTaskGetTickCount();
    uint32_t         bme_ms  = 0;
    uint32_t         wait_ms = BH1750_MEAS_MS;

    /* 0. Diagnóstico só quando algum sensor I2C falhou no ciclo anterior */
    if (s_i2c_missing) {
        (void)i2c_bus_scan(s_bus);
    }

    /* 1. Dispara as conversões, que correm em paralelo */
    r->env_err = bme_start(&bme_ms);
    r->lux_err = bh_start();
    wait_ms = MAX(wait_ms, bme_ms);

    /* 2. Espera única pela mais lenta (aquecedor do BME680, ~170 ms) */
    const TickType_t elapsed = xTaskGetTickCount() - t0;
    if (elapsed < pdMS_TO_TICKS(wait_ms)) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms) - elapsed);
    }

    /* 3. Leituras */
    if (r->env_err == ESP_OK) {
        r->env_err = bme680_read(&r->env);
    }
    if (r->lux_err == ESP_OK) {
        r->lux_err = bh1750_read(&r->lux);
    }
    r->rain_err = s_rain_ok ? rain_sensor_read(&r->rain) : ESP_ERR_INVALID_STATE;
    r->dust_err = s_gp2y_ok ? gp2y_read(&r->dust) : ESP_ERR_INVALID_STATE;
    r->wind_err = s_wind_ok ? wind_sensor_read(&r->wind) : ESP_ERR_INVALID_STATE;
    r->rain_total_mm = s_gauge_ok ? rain_gauge_total_mm() : 0.0f;
}

/** Barramento travado (SDA preso em 0) derruba os dois sensores ao mesmo tempo. */
static void i2c_check(const readings_t *r)
{
    s_i2c_missing = (r->env_err != ESP_OK) || (r->lux_err != ESP_OK);

    if (r->env_err == ESP_OK || r->lux_err == ESP_OK) {
        s_i2c_fail = 0;
        return;
    }

    s_i2c_fail++;
    if (s_i2c_fail == I2C_FAIL_RESET) {
        ESP_LOGW(TAG, "I2C sem resposta ha %d ciclos: reset do barramento (%s)",
                 I2C_FAIL_RESET, esp_err_to_name(i2c_bus_reset(s_bus)));
    } else if (s_i2c_fail >= I2C_FAIL_RESTART) {
        ESP_LOGE(TAG, "I2C sem resposta ha %d ciclos: reiniciando", I2C_FAIL_RESTART);
        vTaskDelay(pdMS_TO_TICKS(100));   /* Deixa o log sair pela UART */
        esp_restart();
    }
}

/* ---------- Pacote ---------- */

static uint16_t u16_sat(float v)
{
    return (!(v > 0.0f)) ? 0 : (v >= 65535.0f) ? UINT16_MAX : (uint16_t)lroundf(v);
}

static int16_t i16_sat(float v)
{
    return (v <= -32767.0f) ? -32767 : (v >= 32767.0f) ? INT16_MAX : (int16_t)lroundf(v);
}

static uint32_t u32_sat(float v)
{
    return (!(v > 0.0f)) ? 0 : (v >= 4294967040.0f) ? UINT32_MAX : (uint32_t)llroundf(v);
}

static void build_packet(const readings_t *r, lora_packet_t *p)
{
    memset(p, 0, sizeof(*p));
    p->seq = s_seq;

    if (r->env_err == ESP_OK) {
        p->valid    |= LORA_VALID_THP;
        p->temp_c100 = i16_sat(r->env.temperature_c * 100.0f);
        p->hum_x100  = u16_sat(r->env.humidity_pct * 100.0f);
        p->press_x10 = u16_sat(r->env.pressure_hpa * 10.0f);
        if (r->env.gas_valid) {
            p->valid  |= LORA_VALID_GAS;
            p->gas_ohm = u32_sat(r->env.gas_ohm);
        }
    }
    if (r->lux_err == ESP_OK) {
        p->valid  |= LORA_VALID_LUX;
        p->lux_x10 = u32_sat(r->lux * 10.0f);
    }
    if (r->dust_err == ESP_OK) {
        p->valid     |= LORA_VALID_DUST;
        p->dust_ugm3  = u16_sat(r->dust.dust_ugm3);
        p->dust_vo_mv = u16_sat((float)r->dust.vo_mv);
        if (r->dust.saturated) {
            p->flags |= LORA_FLAG_DUST_SAT;
        }
    }
    if (r->rain_err == ESP_OK) {
        p->valid  |= LORA_VALID_RAIN;
        p->rain_mv = u16_sat((float)r->rain.mv);
        if (r->rain.wet) {
            p->flags |= LORA_FLAG_RAIN_WET;
        }
    }
    if (r->wind_err == ESP_OK) {
        p->valid   |= LORA_VALID_WIND;
        p->wind_cms = u16_sat(r->wind.avg_ms * 100.0f);
        p->gust_cms = u16_sat(r->wind.gust_ms * 100.0f);
    }
    if (s_gauge_ok) {
        p->valid    |= LORA_VALID_GAUGE;
        p->rain_x100 = u32_sat(r->rain_total_mm * 100.0f);
    }
    lora_packet_finalize(p);
}

/* Valor formatado, ou "--" quando o sensor não leu (bit de valid em 0) */
static const char *fmt(char *buf, size_t n, bool ok, const char *f, double v)
{
    if (!ok) {
        return "--";
    }
    snprintf(buf, n, f, v);
    return buf;
}

static void log_readings(const readings_t *r, const lora_packet_t *p)
{
    char t[12], h[12], pr[12], g[12], lx[12], d[12], vo[12], rn[12], ws[12], wg[12], rt[12];
    const bool thp  = lora_packet_has(p, LORA_VALID_THP);
    const bool rain = lora_packet_has(p, LORA_VALID_RAIN);
    const bool wind = lora_packet_has(p, LORA_VALID_WIND);
    const bool dust = lora_packet_has(p, LORA_VALID_DUST);

    ESP_LOGI(TAG, "#%u valid=0x%02X | T=%sC UR=%s%% P=%shPa gas=%sk | lux=%s", p->seq, p->valid,
             fmt(t,  sizeof(t),  thp, "%.2f", r->env.temperature_c),
             fmt(h,  sizeof(h),  thp, "%.1f", r->env.humidity_pct),
             fmt(pr, sizeof(pr), thp, "%.1f", r->env.pressure_hpa),
             fmt(g,  sizeof(g),  lora_packet_has(p, LORA_VALID_GAS), "%.1f", r->env.gas_ohm / 1000.0),
             fmt(lx, sizeof(lx), lora_packet_has(p, LORA_VALID_LUX), "%.0f", r->lux));
    ESP_LOGI(TAG, "poeira=%s%sug/m3 (Vo=%s mV) | FC-22=%s mV %s | vento=%s raj=%s m/s | chuva=%s mm",
             (dust && r->dust.saturated) ? ">" : "",
             fmt(d,  sizeof(d),  dust, "%.0f", r->dust.dust_ugm3),
             fmt(vo, sizeof(vo), dust, "%.0f", (double)r->dust.vo_mv),
             fmt(rn, sizeof(rn), rain, "%.0f", (double)r->rain.mv),
             rain ? (r->rain.wet ? "molhado" : "seco") : "",
             fmt(ws, sizeof(ws), wind, "%.2f", r->wind.avg_ms),
             fmt(wg, sizeof(wg), wind, "%.2f", r->wind.gust_ms),
             fmt(rt, sizeof(rt), lora_packet_has(p, LORA_VALID_GAUGE), "%.2f", r->rain_total_mm));

    /* Um aviso por sensor que falhou neste ciclo, com o motivo */
    const struct { const char *name; esp_err_t err; } st[] = {
        { "BME680", r->env_err },  { "BH1750", r->lux_err }, { "GP2Y", r->dust_err },
        { "FC-22",  r->rain_err }, { "Vento",  r->wind_err },
    };
    for (size_t i = 0; i < sizeof(st) / sizeof(st[0]); i++) {
        if (st[i].err != ESP_OK) {
            ESP_LOGW(TAG, "%s sem leitura: %s%s", st[i].name, esp_err_to_name(st[i].err),
                     st[i].err == ESP_ERR_INVALID_STATE ? " (nao inicializado)" : "");
        }
    }
}

/* ---------- Loop ---------- */

void station_run(void)
{
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        readings_t    r = { 0 };
        lora_packet_t pkt;

        measure(&r);
        build_packet(&r, &pkt);
        log_readings(&r, &pkt);

        esp_err_t ret = lora_sx1262_transmit(&s_lora, (const uint8_t *)&pkt, sizeof(pkt));
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "LoRa TX OK (%u bytes, CRC 0x%04X)", (unsigned)sizeof(pkt), pkt.crc16);
        } else {
            ESP_LOGE(TAG, "LoRa TX falhou: %s", esp_err_to_name(ret));
        }
        s_seq++;

        i2c_check(&r);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(STATION_TX_PERIOD_S * 1000U));
    }
}
