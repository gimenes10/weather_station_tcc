#include "weather_task.h"
#include "pin_config.h"
#include "i2c_bus.h"
#include "adc_reader.h"
#include "bme280.h"
#include "mpu6050.h"
#include "bh1750.h"
#include "mq_sensor.h"
#include "rain_sensor.h"
#include "oled_ssd1306.h"
#include "lora_sx1262.h"
#include "lora_packet.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>

static const char *TAG = "weather";

#define READING_INTERVAL_MS  5000   /* Intervalo entre leituras */

/* Handles globais dos periféricos */
static i2c_master_bus_handle_t   sensor_bus;
static i2c_master_bus_handle_t   oled_bus;
static adc_oneshot_unit_handle_t adc_handle;

static bme280_handle_t           bme280;
static mpu6050_handle_t          mpu6050;
static bh1750_handle_t           bh1750;
static oled_handle_t             oled;
static lora_sx1262_handle_t      lora;

static bool oled_available = false;

/* ---------- Inicialização ---------- */

esp_err_t weather_station_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Estacao Meteorologica IoT — Init");
    ESP_LOGI(TAG, "  Heltec WiFi LoRa 32 V3 | ESP-IDF v6.0");
    ESP_LOGI(TAG, "========================================");

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

    /* 3. OLED */
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

    /* 4. BME280 */
    ret = bme280_init(&bme280, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init BME280");
        goto err_init;
    }

    /* 5. MPU-6050 */
    ret = mpu6050_init(&mpu6050, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init MPU-6050");
        goto err_init;
    }

    /* 6. BH1750 */
    ret = bh1750_init(&bh1750, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init BH1750");
        goto err_init;
    }

    /* 7. Sensor de chuva */
    ret = rain_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init sensor de chuva");
        goto err_init;
    }

    /* 8. LoRa SX1262 */
    ret = lora_sx1262_init(&lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init LoRa SX1262");
        goto err_init;
    }

    ESP_LOGI(TAG, "Todos os perifericos inicializados com sucesso!");
    return ESP_OK;

err_init:
    return ret;
}

/* ---------- Serialização do pacote LoRa ---------- */

static void build_packet(lora_packet_t *pkt,
                         const bme280_data_t *env, const mpu6050_data_t *imu,
                         float lux, const mq_data_t *mq, const rain_data_t *rain)
{
    /* BME280 */
    pkt->temperature_c = env->temperature_c;
    pkt->humidity_pct  = env->humidity_pct;
    pkt->pressure_hpa  = env->pressure_hpa;

    /* BH1750 */
    pkt->lux = lux;

    /* MQ-7 */
    pkt->mq7_raw     = (int16_t)mq->mq7_raw;
    pkt->mq7_voltage = mq->mq7_voltage;

    /* MQ-2 */
    pkt->mq2_raw     = (int16_t)mq->mq2_raw;
    pkt->mq2_voltage = mq->mq2_voltage;

    /* Chuva */
    pkt->rain_raw     = (int16_t)rain->raw;
    pkt->rain_voltage = rain->voltage;
    pkt->rain_status  = rain->is_raining ? 1 : 0;

    /* MPU-6050 */
    pkt->accel_x    = imu->accel_x;
    pkt->accel_y    = imu->accel_y;
    pkt->accel_z    = imu->accel_z;
    pkt->gyro_x     = imu->gyro_x;
    pkt->gyro_y     = imu->gyro_y;
    pkt->gyro_z     = imu->gyro_z;
    pkt->imu_temp_c = imu->temperature_c;

    /* Finaliza: preenche magic + calcula CRC */
    lora_packet_finalize(pkt);
}

/* ---------- Formata e exibe no OLED ---------- */

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

/* ---------- Log serial ---------- */

static void log_serial(const bme280_data_t *env, const mpu6050_data_t *imu,
                       float lux, const mq_data_t *mq, const rain_data_t *rain)
{
    ESP_LOGI(TAG, "--- Leitura dos sensores ---");
    ESP_LOGI(TAG, "BME280  | Temp=%.2f C | Umid=%.1f %% | Press=%.2f hPa",
             env->temperature_c, env->humidity_pct, env->pressure_hpa);
    ESP_LOGI(TAG, "BH1750  | Lux=%.1f", lux);
    ESP_LOGI(TAG, "MQ-7    | Raw=%d | Voltage=%.2f V", mq->mq7_raw, mq->mq7_voltage);
    ESP_LOGI(TAG, "MQ-2    | Raw=%d | Voltage=%.2f V", mq->mq2_raw, mq->mq2_voltage);
    ESP_LOGI(TAG, "Chuva   | Raw=%d | Voltage=%.2f V | %s",
             rain->raw, rain->voltage, rain->is_raining ? "CHOVENDO" : "SECO");
    ESP_LOGI(TAG, "MPU6050 | Accel=(%.2f, %.2f, %.2f) g",
             imu->accel_x, imu->accel_y, imu->accel_z);
    ESP_LOGI(TAG, "MPU6050 | Gyro=(%.1f, %.1f, %.1f) deg/s",
             imu->gyro_x, imu->gyro_y, imu->gyro_z);
    ESP_LOGI(TAG, "MPU6050 | Temp interna=%.1f C", imu->temperature_c);
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
    lora_packet_t  pkt  = {0};
    bool           lora_ok = false;

    /* Aquecimento dos sensores MQ */
    ESP_LOGI(TAG, "Aguardando aquecimento dos sensores MQ (20s)...");
    if (oled_available) {
        oled_clear(&oled);
        oled_draw_text(&oled, 0, 3, "Aquecendo MQ...");
        oled_draw_text(&oled, 0, 4, "Aguarde 20s");
        oled_update(&oled);
    }
    vTaskDelay(pdMS_TO_TICKS(20000));

    ESP_LOGI(TAG, "Iniciando leituras a cada %d ms", READING_INTERVAL_MS);

    while (1) {
        /* Lê todos os sensores — falhas individuais não param o loop */
        if (bme280_read(&bme280, &env) != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler BME280");
        }

        if (mpu6050_read(&mpu6050, &imu) != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler MPU-6050");
        }

        if (bh1750_read(&bh1750, &lux) != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler BH1750");
        }

        if (mq_sensor_read(adc_handle, &mq) != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler MQ sensors");
        }

        if (rain_sensor_read(adc_handle, &rain) != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao ler sensor de chuva");
        }

        /* Log serial */
        log_serial(&env, &imu, lux, &mq, &rain);

        /* Serializa e transmite via LoRa */
        build_packet(&pkt, &env, &imu, lux, &mq, &rain);

        lora_ok = (lora_sx1262_transmit(&lora, (const uint8_t *)&pkt,
                                        sizeof(pkt)) == ESP_OK);
        if (lora_ok) {
            ESP_LOGI(TAG, "LoRa TX OK (%zu bytes, CRC=0x%04X)",
                     sizeof(pkt), pkt.crc16);
        } else {
            ESP_LOGW(TAG, "LoRa TX falhou");
        }

        /* Atualiza display */
        update_oled(&env, &imu, lux, &mq, &rain, lora_ok);

        vTaskDelay(pdMS_TO_TICKS(READING_INTERVAL_MS));
    }
}
