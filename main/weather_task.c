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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <math.h>

static const char *TAG = "weather";

#define READING_INTERVAL_MS  5000   /* Intervalo entre leituras */

/* Handles globais dos periféricos */
static i2c_master_bus_handle_t  sensor_bus;
static i2c_master_bus_handle_t  oled_bus;
static adc_oneshot_unit_handle_t adc_handle;

static bme280_handle_t   bme280;
static mpu6050_handle_t  mpu6050;
static bh1750_handle_t   bh1750;
static oled_handle_t     oled;

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
        goto err_i2c;
    }

    /* 2. ADC */
    ret = adc_reader_init(&adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init ADC");
        goto err_adc;
    }

    /* 3. OLED (inicializa antes dos sensores para mostrar status) */
    ret = oled_init(&oled, oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED nao inicializado — continuando sem display");
        /* Não é fatal: segue sem display */
    } else {
        oled_clear(&oled);
        oled_draw_text(&oled, 0, 0, "Estacao Meteo");
        oled_draw_text(&oled, 0, 1, "Inicializando...");
        oled_update(&oled);
    }

    /* 4. BME280 */
    ret = bme280_init(&bme280, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init BME280");
        goto err_bme;
    }

    /* 5. MPU-6050 */
    ret = mpu6050_init(&mpu6050, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init MPU-6050");
        goto err_mpu;
    }

    /* 6. BH1750 */
    ret = bh1750_init(&bh1750, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init BH1750");
        goto err_bh;
    }

    /* 7. Sensor de chuva (GPIO power control) */
    ret = rain_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init sensor de chuva");
        goto err_rain;
    }

    ESP_LOGI(TAG, "Todos os sensores inicializados com sucesso!");
    return ESP_OK;

/* Cadeia de cleanup em caso de erro */
err_rain:
err_bh:
err_mpu:
err_bme:
err_adc:
err_i2c:
    return ret;
}

/* ---------- Formata e exibe no OLED ---------- */

static void update_oled(const bme280_data_t *env, const mpu6050_data_t *imu,
                        float lux, const mq_data_t *mq, const rain_data_t *rain)
{
    char line[22]; /* 21 chars por linha (128/6) + null */

    oled_clear(&oled);

    /* Linha 0: Temperatura + Umidade */
    snprintf(line, sizeof(line), "T:%.1fC  H:%.0f%%", env->temperature_c, env->humidity_pct);
    oled_draw_text(&oled, 0, 0, line);

    /* Linha 1: Pressão */
    snprintf(line, sizeof(line), "P:%.1f hPa", env->pressure_hpa);
    oled_draw_text(&oled, 0, 1, line);

    /* Linha 2: Luminosidade */
    snprintf(line, sizeof(line), "Lux:%.0f", lux);
    oled_draw_text(&oled, 0, 2, line);

    /* Linha 3: MQ-7 (CO) e MQ-2 (GLP) */
    snprintf(line, sizeof(line), "CO:%d GLP:%d", mq->mq7_raw, mq->mq2_raw);
    oled_draw_text(&oled, 0, 3, line);

    /* Linha 4: Chuva */
    snprintf(line, sizeof(line), "Chuva:%s (%d)", rain->is_raining ? "SIM" : "NAO", rain->raw);
    oled_draw_text(&oled, 0, 4, line);

    /* Linha 5: Acelerômetro */
    snprintf(line, sizeof(line), "Ax:%.1f Ay:%.1f Az:%.1f",
             imu->accel_x, imu->accel_y, imu->accel_z);
    oled_draw_text(&oled, 0, 5, line);

    /* Linha 6: Giroscópio */
    snprintf(line, sizeof(line), "Gx:%.0f Gy:%.0f Gz:%.0f",
             imu->gyro_x, imu->gyro_y, imu->gyro_z);
    oled_draw_text(&oled, 0, 6, line);

    /* Linha 7: Status */
    oled_draw_text(&oled, 0, 7, ">> Estacao Meteo v1");

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

    /* Aquecimento dos sensores MQ (~20s recomendado no datasheet) */
    ESP_LOGI(TAG, "Aguardando aquecimento dos sensores MQ (20s)...");
    oled_clear(&oled);
    oled_draw_text(&oled, 0, 3, "Aquecendo MQ...");
    oled_draw_text(&oled, 0, 4, "Aguarde 20s");
    oled_update(&oled);
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

        /* Exibe resultados */
        log_serial(&env, &imu, lux, &mq, &rain);
        update_oled(&env, &imu, lux, &mq, &rain);

        vTaskDelay(pdMS_TO_TICKS(READING_INTERVAL_MS));
    }
}
