#include "weather_task.h"
#include "pin_config.h"
#include "i2c_bus.h"
#include "adc_reader.h"
#include "bme280.h"
#include "mpu6050.h"
#include "bh1750.h"
#include "mq_sensor.h"
#include "rain_sensor.h"
#include "wind_sensor.h"
#include "rain_gauge.h"
#include "oled_ssd1306.h"
#include "lora_sx1262.h"
#include "lora_packet.h"
#include "sensor_health.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "weather";

#define READING_INTERVAL_MS  5000   /* Intervalo entre leituras */
#define DIAG_FAIL_CYCLES     3      /* Ciclos seguidos sem nenhum I2C OK para declarar falha */
#define ENABLE_MPU6050       0      /* MPU-6050 fora do projeto — código mantido, campos zerados no pacote */

/*
 * Escada de recuperação do barramento I2C, em ciclos após a falha declarada
 * (1 ciclo = READING_INTERVAL_MS):
 *   0                  → degrau 1: i2c_master_bus_reset()
 *   3, 15, 27, ...     → degrau 2: destrói e recria o barramento + sensores
 *   RECOVER_RESTART    → degrau 3: esp_restart()
 */
#define RECOVER_RECREATE_FIRST   3      /* ~15 s após a falha */
#define RECOVER_RECREATE_EVERY   12     /* ~60 s entre recriações */
#define RECOVER_RESTART          60     /* ~5 min após a falha */
#define RECOVER_MAX_RESTARTS     3      /* Após isso, só degrau 2 (evita loop de boot) */

/* Tag mantida ativa enquanto os demais logs estão silenciados */
static const char *TAG_REC = "i2c_recover";

/* Sobrevive ao esp_restart() — registra reinicializações por falha I2C */
#define RTC_I2C_MAGIC            0x12C0FA11U
static RTC_NOINIT_ATTR uint32_t rtc_i2c_magic;
static RTC_NOINIT_ATTR uint32_t rtc_i2c_restarts;

typedef enum {
    RECOVER_NONE = 0,
    RECOVER_BUS_RESET,
    RECOVER_BUS_RECREATE,
} recover_stage_t;

/* Diagnóstico de falha do barramento I2C */
typedef struct {
    esp_err_t err_bme;
#if ENABLE_MPU6050
    esp_err_t err_mpu;
#endif
    esp_err_t err_bh;
    uint32_t  fail_streak;   /* Ciclos consecutivos sem nenhum sensor I2C OK */
    bool      i2c_down;      /* Falha declarada — logs silenciados */
    recover_stage_t stage;   /* Último degrau de recuperação aplicado */
    bool      ever_ok;       /* Houve ao menos uma leitura I2C válida desde o boot */
    float     last_temp_c;   /* Última leitura válida do BME280 */
    float     last_hum_pct;
    uint32_t  last_ok_s;     /* Uptime da última leitura I2C válida */
} diag_t;

static diag_t                      diag;
static temperature_sensor_handle_t tsens;

/* Handles globais dos periféricos */
static i2c_master_bus_handle_t   sensor_bus;
static i2c_master_bus_handle_t   oled_bus;
static adc_oneshot_unit_handle_t adc_handle;

static bme280_handle_t           bme280;
#if ENABLE_MPU6050
static mpu6050_handle_t          mpu6050;
#endif
static bh1750_handle_t           bh1750;
// static oled_handle_t             oled;   /* OLED desativado em produção */
static lora_sx1262_handle_t      lora;
static sensor_health_t           health;

/* Sensores de pulso: sem hot-plug (falha de init = GPIO/ISR, não fiação) */
static bool wind_ok;
static bool gauge_ok;

// static bool oled_available = false;  /* OLED desativado em produção */

/* ---------- Inicialização ---------- */

esp_err_t weather_station_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Estacao Meteorologica IoT — Init");
    ESP_LOGI(TAG, "  Heltec WiFi LoRa 32 V3 | ESP-IDF v6.1");
    ESP_LOGI(TAG, "========================================");

    /* Reinicialização provocada pela recuperação de I2C? */
    if (esp_reset_reason() == ESP_RST_SW && rtc_i2c_magic == RTC_I2C_MAGIC) {
        ESP_LOGW(TAG, "Boot apos reinicio por falha I2C (total: %lu)",
                 (unsigned long)rtc_i2c_restarts);
    } else {
        rtc_i2c_magic    = RTC_I2C_MAGIC;
        rtc_i2c_restarts = 0;
    }

    /* 1. Barramentos I2C */
    ret = i2c_bus_init(&sensor_bus, &oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init I2C");
        goto err_init;
    }

    /* 2. ADC */
    ret = adc_reader_init(&adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init ADC");
        goto err_init;
    }

    /* 3. OLED — desativado em produção (sem init o SSD1306 fica em display OFF) */
#if 0
    ret = oled_init(&oled, oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED nao inicializado — continuando sem display");
    } else {
        oled_available = true;
        oled_clear(&oled);
        oled_draw_text(&oled, 0, 0, "Estacao Meteo");
        oled_draw_text(&oled, 0, 1, "Inicializando...");
        oled_update(&oled);
    }
#endif

    /*
     * 4-7. Sensores — NÃO fatais. Um sensor ausente fica com dev == NULL,
     * suas leituras retornam ESP_ERR_INVALID_STATE e o sensor_health
     * tenta reinicializá-lo a cada HEALTH_RETRY_CYCLES (hot-plug).
     */
    uint8_t sensors_failed = 0;

    if (bme280_init(&bme280, sensor_bus) != ESP_OK) {
        ESP_LOGW(TAG, "BME280 offline — continuando sem ele");
        sensors_failed++;
    }

#if ENABLE_MPU6050
    if (mpu6050_init(&mpu6050, sensor_bus) != ESP_OK) {
        ESP_LOGW(TAG, "MPU-6050 offline — continuando sem ele");
        sensors_failed++;
    }
#endif

    if (bh1750_init(&bh1750, sensor_bus) != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 offline — continuando sem ele");
        sensors_failed++;
    }

    if (rain_sensor_init() != ESP_OK) {
        ESP_LOGW(TAG, "Sensor de chuva offline — continuando sem ele");
        sensors_failed++;
    }

    wind_ok = (wind_sensor_init() == ESP_OK);
    if (!wind_ok) {
        ESP_LOGW(TAG, "Anemometro offline — continuando sem ele");
        sensors_failed++;
    }

    gauge_ok = (rain_gauge_init() == ESP_OK);
    if (!gauge_ok) {
        ESP_LOGW(TAG, "Pluviometro offline — continuando sem ele");
        sensors_failed++;
    }

    /* 8. LoRa SX1262 — fatal: sem rádio o nó não tem função */
    ret = lora_sx1262_init(&lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init LoRa SX1262");
        goto err_init;
    }

    /* 9. Sensor Health */
    sensor_health_init(&health);

    /* 10. Sensor de temperatura interno do ESP32-S3 — não fatal (só diagnóstico) */
    temperature_sensor_config_t ts_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&ts_cfg, &tsens) != ESP_OK) {
        ESP_LOGW(TAG, "Sensor de temperatura interno indisponivel");
        tsens = NULL;
    } else if (temperature_sensor_enable(tsens) != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao habilitar sensor de temperatura interno");
        temperature_sensor_uninstall(tsens);
        tsens = NULL;
    }

    if (sensors_failed == 0) {
        ESP_LOGI(TAG, "Todos os perifericos inicializados com sucesso!");
    } else {
        ESP_LOGW(TAG, "Init concluido com %u sensor(es) offline", sensors_failed);
    }
    return ESP_OK;

err_init:
    return ret;
}

/* ---------- Serialização do pacote LoRa ---------- */

/**
 * @brief Monta o pacote v2. Só preenche os campos dos sensores com bit
 *        em `valid`; os demais ficam zerados e o gateway publica `null`.
 */
static void build_packet(lora_packet_t *pkt, uint8_t valid,
                         const bme280_data_t *env, float lux,
                         const mq_data_t *mq, const rain_data_t *rain,
                         const wind_data_t *wind, float rain_total_mm)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->valid = valid;

    if (valid & LORA_VALID_BME280) {
        pkt->temperature_c = env->temperature_c;
        pkt->humidity_pct  = env->humidity_pct;
        pkt->pressure_hpa  = env->pressure_hpa;
    }

    if (valid & LORA_VALID_BH1750) {
        pkt->lux = lux;
    }

    if (valid & LORA_VALID_MQ7) {
        pkt->mq7_raw     = (int16_t)mq->mq7_raw;
        pkt->mq7_voltage = mq->mq7_voltage;
    }

    if (valid & LORA_VALID_MQ2) {
        pkt->mq2_raw     = (int16_t)mq->mq2_raw;
        pkt->mq2_voltage = mq->mq2_voltage;
    }

    if (valid & LORA_VALID_RAIN) {
        pkt->rain_raw     = (int16_t)rain->raw;
        pkt->rain_voltage = rain->voltage;
        pkt->rain_status  = rain->is_raining ? 1 : 0;
    }

    if (valid & LORA_VALID_WIND) {
        pkt->wind_avg_ms  = wind->avg_ms;
        pkt->wind_gust_ms = wind->gust_ms;
    }

    if (valid & LORA_VALID_GAUGE) {
        pkt->rain_total_mm = rain_total_mm;
    }

    /* Finaliza: magic + versão + CRC */
    lora_packet_finalize(pkt);
}

/* ---------- Formata e exibe no OLED (desativado em produção) ---------- */

#if 0
static void update_oled(const bme280_data_t *env, const mpu6050_data_t *imu,
                        float lux, const mq_data_t *mq, const rain_data_t *rain,
                        bool lora_ok)
{
    if (!oled_available) { return; }

    char line[22]; /* 21 chars max (128/6) + null */

    oled_clear(&oled);

    snprintf(line, sizeof(line), "T:%.1fC  H:%.0f%%", env->temperature_c, env->humidity_pct);
    oled_draw_text(&oled, 0, 0, line);

    snprintf(line, sizeof(line), "P:%.1f hPa", env->pressure_hpa);
    oled_draw_text(&oled, 0, 1, line);

    snprintf(line, sizeof(line), "Lux:%.0f", lux);
    oled_draw_text(&oled, 0, 2, line);

    snprintf(line, sizeof(line), "CO:%d GLP:%d", mq->mq7_raw, mq->mq2_raw);
    oled_draw_text(&oled, 0, 3, line);

    snprintf(line, sizeof(line), "Chuva:%s (%d)", rain->is_raining ? "SIM" : "NAO", rain->raw);
    oled_draw_text(&oled, 0, 4, line);

    snprintf(line, sizeof(line), "Ax:%.1f Ay:%.1f Az:%.1f",
             imu->accel_x, imu->accel_y, imu->accel_z);
    oled_draw_text(&oled, 0, 5, line);

    snprintf(line, sizeof(line), "Gx:%.0f Gy:%.0f Gz:%.0f",
             imu->gyro_x, imu->gyro_y, imu->gyro_z);
    oled_draw_text(&oled, 0, 6, line);

    snprintf(line, sizeof(line), "LoRa:%s | v1.0", lora_ok ? "TX OK" : "FALHA");
    oled_draw_text(&oled, 0, 7, line);

    oled_update(&oled);
}
#endif

/* ---------- Log serial ---------- */

static void log_serial(const bme280_data_t *env, const mpu6050_data_t *imu,
                       float lux, const mq_data_t *mq, const rain_data_t *rain,
                       const wind_data_t *wind, float rain_total_mm)
{
    (void)imu;  /* Print do MPU-6050 comentado abaixo */

    ESP_LOGI(TAG, "--- Leitura dos sensores ---");
    ESP_LOGI(TAG, "BME280  | Temp=%.2f C | Umid=%.1f %% | Press=%.2f hPa",
             env->temperature_c, env->humidity_pct, env->pressure_hpa);
    ESP_LOGI(TAG, "BH1750  | Lux=%.1f", lux);
    ESP_LOGI(TAG, "MQ-7    | Raw=%d | Voltage=%.2f V | %s (swing=%d mV)",
             mq->mq7_raw, mq->mq7_voltage,
             mq->mq7_present ? "conectado" : "DESCONECTADO", mq->mq7_swing_mv);
    ESP_LOGI(TAG, "MQ-2    | Raw=%d | Voltage=%.2f V | %s (swing=%d mV)",
             mq->mq2_raw, mq->mq2_voltage,
             mq->mq2_present ? "conectado" : "DESCONECTADO", mq->mq2_swing_mv);
    ESP_LOGI(TAG, "Chuva   | Raw=%d | Voltage=%.2f V | %s | %s (swing=%d mV)",
             rain->raw, rain->voltage, rain->is_raining ? "CHOVENDO" : "SECO",
             rain->present ? "conectado" : "DESCONECTADO", rain->swing_mv);
    ESP_LOGI(TAG, "Vento   | Media=%.2f m/s | Rajada=%.2f m/s", wind->avg_ms, wind->gust_ms);
    ESP_LOGI(TAG, "Pluvio  | Acumulado=%.2f mm", rain_total_mm);
    // ESP_LOGI(TAG, "MPU6050 | Accel=(%.2f, %.2f, %.2f) g",
    //          imu->accel_x, imu->accel_y, imu->accel_z);
    // ESP_LOGI(TAG, "MPU6050 | Gyro=(%.1f, %.1f, %.1f) deg/s",
    //          imu->gyro_x, imu->gyro_y, imu->gyro_z);
    // ESP_LOGI(TAG, "MPU6050 | Temp interna=%.1f C", imu->temperature_c);
}

/* ---------- Diagnóstico ---------- */

static uint32_t uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

static float chip_temp_c(void)
{
    float t = NAN;

    if (tsens != NULL) {
        temperature_sensor_get_celsius(tsens, &t);
    }
    return t;
}

/** Linha de diagnóstico a cada ciclo (estado normal). */
static void log_diag(void)
{
    ESP_LOGI(TAG, "DIAG    | up=%lus | chip=%.1fC | SDA=%d SCL=%d | BME=%s BH=%s",
             (unsigned long)uptime_s(), chip_temp_c(),
             gpio_get_level(SENSOR_I2C_SDA), gpio_get_level(SENSOR_I2C_SCL),
             esp_err_to_name(diag.err_bme), esp_err_to_name(diag.err_bh));
}

/** Bloco único impresso na transição OK → falha. */
static void log_failure_block(void)
{
    ESP_LOGE(TAG, "=========== FALHA DETECTADA ===========");
    if (diag.ever_ok) {
        ESP_LOGE(TAG, "Uptime            : %lus (ultima leitura I2C OK em %lus)",
                 (unsigned long)uptime_s(), (unsigned long)diag.last_ok_s);
        ESP_LOGE(TAG, "Ultimo BME280 OK  : %.2f C | %.1f %%",
                 diag.last_temp_c, diag.last_hum_pct);
    } else {
        ESP_LOGE(TAG, "Uptime            : %lus (nenhuma leitura I2C OK desde o boot)",
                 (unsigned long)uptime_s());
    }
    ESP_LOGE(TAG, "Temp. chip ESP32  : %.1f C", chip_temp_c());
    ESP_LOGE(TAG, "Nivel SDA / SCL   : %d / %d",
             gpio_get_level(SENSOR_I2C_SDA), gpio_get_level(SENSOR_I2C_SCL));
    ESP_LOGE(TAG, "Erro BME280       : %s", esp_err_to_name(diag.err_bme));
    ESP_LOGE(TAG, "Erro BH1750       : %s", esp_err_to_name(diag.err_bh));
    ESP_LOGE(TAG, "Resets (BME/BH)   : %lu / %lu",
             (unsigned long)health.bme280.resets,
             (unsigned long)health.bh1750.resets);
    ESP_LOGE(TAG, "Logs silenciados ate a recuperacao do barramento");
    ESP_LOGE(TAG, "=======================================");
}

/**
 * @brief Remove os dispositivos I2C do barramento (pré-requisito para
 *        destruí-lo). Sensores ficam com dev == NULL (offline).
 */
static void i2c_detach_sensors(void)
{
    if (bme280.dev != NULL) {
        i2c_master_bus_rm_device(bme280.dev);
        bme280.dev = NULL;
    }
#if ENABLE_MPU6050
    if (mpu6050.dev != NULL) {
        i2c_master_bus_rm_device(mpu6050.dev);
        mpu6050.dev = NULL;
    }
#endif
    if (bh1750.dev != NULL) {
        i2c_master_bus_rm_device(bh1750.dev);
        bh1750.dev = NULL;
    }
}

/**
 * @brief Aplica o degrau de recuperação correspondente ao tempo em falha.
 *        Logs estão silenciados — o resultado aparece no "RECUPERADO".
 */
static void i2c_recover(void)
{
    uint32_t  n = diag.fail_streak - DIAG_FAIL_CYCLES;   /* Ciclos desde a falha */
    esp_err_t ret;

    if (n == 0) {
        /* Degrau 1: 9 pulsos no SCL + reset do controlador */
        ret = i2c_master_bus_reset(sensor_bus);
        ESP_LOGW(TAG_REC, "Degrau 1 (bus reset): %s", esp_err_to_name(ret));
        diag.stage = RECOVER_BUS_RESET;
        return;
    }

    if (n >= RECOVER_RESTART && rtc_i2c_restarts < RECOVER_MAX_RESTARTS) {
        /* Degrau 3: reinicia o ESP32 — contador sobrevive em RTC */
        rtc_i2c_magic = RTC_I2C_MAGIC;
        rtc_i2c_restarts++;
        ESP_LOGE(TAG_REC, "Degrau 3: reiniciando ESP32 (%lu/%d)",
                 (unsigned long)rtc_i2c_restarts, RECOVER_MAX_RESTARTS);
        vTaskDelay(pdMS_TO_TICKS(100));   /* Deixa o log sair pela UART */
        esp_restart();
    }

    if (n >= RECOVER_RECREATE_FIRST &&
        ((n - RECOVER_RECREATE_FIRST) % RECOVER_RECREATE_EVERY) == 0) {
        /* Degrau 2: destrói e recria o barramento, reinicializa sensores */
        i2c_detach_sensors();
        ret = i2c_bus_sensor_recreate(&sensor_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_REC, "Degrau 2 (recriar barramento): %s", esp_err_to_name(ret));
        } else {
            esp_err_t r_bme = bme280_init(&bme280, sensor_bus);
#if ENABLE_MPU6050
            mpu6050_init(&mpu6050, sensor_bus);
#endif
            esp_err_t r_bh  = bh1750_init(&bh1750, sensor_bus);
            ESP_LOGW(TAG_REC, "Degrau 2 (barramento recriado): BME280=%s BH1750=%s",
                     esp_err_to_name(r_bme), esp_err_to_name(r_bh));
        }
        diag.stage = RECOVER_BUS_RECREATE;
    }
}

static const char *stage_name(recover_stage_t s)
{
    switch (s) {
    case RECOVER_BUS_RESET:    return "bus reset";
    case RECOVER_BUS_RECREATE: return "recriacao do barramento";
    default:                   return "espontanea";
    }
}

/**
 * @brief Atualiza o estado de falha do barramento I2C.
 *
 * Após DIAG_FAIL_CYCLES ciclos sem nenhum sensor I2C OK, imprime o bloco
 * de falha uma única vez, silencia todos os logs e inicia a escada de
 * recuperação. Na primeira leitura válida seguinte, restaura os logs e
 * registra qual degrau resolveu.
 */
static void diag_update(const bme280_data_t *env)
{
    bool any_ok = (diag.err_bme == ESP_OK)
#if ENABLE_MPU6050
               || (diag.err_mpu == ESP_OK)
#endif
               || (diag.err_bh  == ESP_OK);

    if (diag.err_bme == ESP_OK) {
        diag.last_temp_c  = env->temperature_c;
        diag.last_hum_pct = env->humidity_pct;
    }

    if (any_ok) {
        diag.ever_ok   = true;
        diag.last_ok_s = uptime_s();
        if (diag.i2c_down) {
            esp_log_level_set("*", CONFIG_LOG_DEFAULT_LEVEL);
            ESP_LOGW(TAG, "=== RECUPERADO apos %lu ciclos via %s (up=%lus, chip=%.1fC) ===",
                     (unsigned long)diag.fail_streak, stage_name(diag.stage),
                     (unsigned long)uptime_s(), chip_temp_c());
            diag.i2c_down = false;
        }
        diag.stage       = RECOVER_NONE;
        diag.fail_streak = 0;
        return;
    }

    diag.fail_streak++;
    if (diag.fail_streak < DIAG_FAIL_CYCLES) {
        return;
    }

    if (!diag.i2c_down) {
        log_failure_block();
        esp_log_level_set("*", ESP_LOG_NONE);
        esp_log_level_set(TAG_REC, ESP_LOG_INFO);   /* Recuperação continua visível */
        diag.i2c_down = true;
    }

    i2c_recover();
}

/* ---------- Task principal ---------- */

void weather_task(void *arg)
{
    (void)arg;

    bme280_data_t  env  = {0};
    mpu6050_data_t imu  = {0};
    float          lux  = 0.0f;
    mq_data_t      mq   = {0};
    rain_data_t    rain = {0};
    wind_data_t    wind = {0};
    float          rain_total_mm = 0.0f;
    lora_packet_t  pkt  = {0};
    bool           lora_ok = false;

    /* Aquecimento dos sensores MQ */
    ESP_LOGI(TAG, "Aguardando aquecimento dos sensores MQ (20s)...");
    // if (oled_available) {
    //     oled_clear(&oled);
    //     oled_draw_text(&oled, 0, 3, "Aquecendo MQ...");
    //     oled_draw_text(&oled, 0, 4, "Aguarde 20s");
    //     oled_update(&oled);
    // }
    vTaskDelay(pdMS_TO_TICKS(20000));

    ESP_LOGI(TAG, "Iniciando leituras a cada %d ms", READING_INTERVAL_MS);

    while (1) {
        /*
         * Lê todos os sensores — falhas individuais não param o loop.
         * Em falha os dados são zerados para não transmitir valor antigo.
         */
        diag.err_bme = bme280_read(&bme280, &env);
        if (diag.err_bme != ESP_OK) {
            ESP_LOGW(TAG, "BME280 sem leitura");
            memset(&env, 0, sizeof(env));
        }

#if ENABLE_MPU6050
        diag.err_mpu = mpu6050_read(&mpu6050, &imu);
        if (diag.err_mpu != ESP_OK) {
            ESP_LOGW(TAG, "MPU-6050 sem leitura");
            memset(&imu, 0, sizeof(imu));
        }
#endif

        diag.err_bh = bh1750_read(&bh1750, &lux);
        if (diag.err_bh != ESP_OK) {
            ESP_LOGW(TAG, "BH1750 sem leitura");
            lux = 0.0f;
        }

        /*
         * Recuperação individual só enquanto o barramento está saudável —
         * com todos os I2C falhando, quem age é a escada do diag_update().
         */
        if (!diag.i2c_down) {
            sensor_health_check_bme280(&health, &bme280, diag.err_bme == ESP_OK, sensor_bus);
#if ENABLE_MPU6050
            sensor_health_check_mpu6050(&health, &mpu6050, diag.err_mpu == ESP_OK, sensor_bus);
#endif
            sensor_health_check_bh1750(&health, &bh1750, diag.err_bh == ESP_OK, sensor_bus);
        }

        esp_err_t err_mq = mq_sensor_read(adc_handle, &mq);
        if (err_mq != ESP_OK) {
            ESP_LOGW(TAG, "MQ sem leitura");
            memset(&mq, 0, sizeof(mq));
        }

        esp_err_t err_rain = rain_sensor_read(adc_handle, &rain);
        if (err_rain != ESP_OK) {
            ESP_LOGW(TAG, "Chuva sem leitura");
            memset(&rain, 0, sizeof(rain));
        }

        esp_err_t err_wind = wind_ok ? wind_sensor_read(&wind) : ESP_ERR_INVALID_STATE;
        if (err_wind != ESP_OK) {
            memset(&wind, 0, sizeof(wind));
        }

        rain_total_mm = gauge_ok ? rain_gauge_total_mm() : 0.0f;

        /* Bitmask de validade do pacote */
        uint8_t valid = 0;
        if (diag.err_bme == ESP_OK)                  { valid |= LORA_VALID_BME280; }
        if (diag.err_bh  == ESP_OK)                  { valid |= LORA_VALID_BH1750; }
        if (err_mq == ESP_OK && mq.mq7_present)      { valid |= LORA_VALID_MQ7; }
        if (err_mq == ESP_OK && mq.mq2_present)      { valid |= LORA_VALID_MQ2; }
        if (err_rain == ESP_OK && rain.present)      { valid |= LORA_VALID_RAIN; }
        if (err_wind     == ESP_OK) { valid |= LORA_VALID_WIND; }
        if (gauge_ok)               { valid |= LORA_VALID_GAUGE; }

        /* Log serial + diagnóstico */
        log_serial(&env, &imu, lux, &mq, &rain, &wind, rain_total_mm);
        log_diag();

        /* Serializa e transmite via LoRa */
        build_packet(&pkt, valid, &env, lux, &mq, &rain, &wind, rain_total_mm);

        lora_ok = (lora_sx1262_transmit(&lora, (const uint8_t *)&pkt,
                                        sizeof(pkt)) == ESP_OK);
        if (lora_ok) {
            ESP_LOGI(TAG, "LoRa TX OK (%zu bytes, valid=0x%02X, CRC=0x%04X)",
                     sizeof(pkt), valid, pkt.crc16);
        } else {
            ESP_LOGW(TAG, "LoRa TX falhou");
        }

        /* Atualiza display — desativado em produção */
        // update_oled(&env, &imu, lux, &mq, &rain, lora_ok);

        /* Detecta falha / recuperação do barramento I2C */
        diag_update(&env);

        vTaskDelay(pdMS_TO_TICKS(READING_INTERVAL_MS));
    }
}
