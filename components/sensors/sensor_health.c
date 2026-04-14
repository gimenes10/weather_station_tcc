#include "sensor_health.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "health";

#define FLOAT_EQ(a, b) (fabsf((a) - (b)) < 0.001f)

void sensor_health_init(sensor_health_t *health)
{
    health->bme280_stale_count  = 0;
    health->mpu6050_stale_count = 0;
    health->bh1750_stale_count  = 0;

    health->last_bme_temp  = -999.0f;
    health->last_bme_hum   = -999.0f;
    health->last_bme_press = -999.0f;
    health->last_mpu_ax    = -999.0f;
    health->last_mpu_ay    = -999.0f;
    health->last_mpu_az    = -999.0f;
    health->last_lux       = -999.0f;

    health->bme280_resets  = 0;
    health->mpu6050_resets = 0;
    health->bh1750_resets  = 0;
}

/* ---------- BME280 ---------- */

bool sensor_health_check_bme280(sensor_health_t *health,
                                 bme280_handle_t *sensor,
                                 const bme280_data_t *data,
                                 i2c_master_bus_handle_t bus)
{
    bool stale = FLOAT_EQ(data->temperature_c, health->last_bme_temp)
              && FLOAT_EQ(data->humidity_pct,   health->last_bme_hum)
              && FLOAT_EQ(data->pressure_hpa,   health->last_bme_press);

    bool invalid = (data->pressure_hpa < 800.0f || data->pressure_hpa > 1100.0f);

    if (stale || invalid) {
        health->bme280_stale_count++;
        if (invalid) {
            ESP_LOGW(TAG, "BME280: pressao invalida (%.1f hPa) — leitura %d/%d",
                     data->pressure_hpa, health->bme280_stale_count, HEALTH_STALE_THRESHOLD);
        } else {
            ESP_LOGW(TAG, "BME280: dados repetidos — leitura %d/%d",
                     health->bme280_stale_count, HEALTH_STALE_THRESHOLD);
        }
    } else {
        health->bme280_stale_count = 0;
    }

    health->last_bme_temp  = data->temperature_c;
    health->last_bme_hum   = data->humidity_pct;
    health->last_bme_press = data->pressure_hpa;

    if (health->bme280_stale_count >= HEALTH_STALE_THRESHOLD) {
        ESP_LOGW(TAG, "BME280: reinicializando sensor (reset #%lu)...",
                 (unsigned long)++health->bme280_resets);
        health->bme280_stale_count = 0;

        i2c_master_bus_rm_device(sensor->dev);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (bme280_init(sensor, bus) == ESP_OK) {
            ESP_LOGI(TAG, "BME280: reinicializado com sucesso");
            return true;
        }
        ESP_LOGE(TAG, "BME280: falha na reinicializacao");
    }

    return false;
}

/* ---------- MPU-6050 ---------- */

bool sensor_health_check_mpu6050(sensor_health_t *health,
                                  mpu6050_handle_t *sensor,
                                  const mpu6050_data_t *data,
                                  i2c_master_bus_handle_t bus)
{
    bool all_zero = FLOAT_EQ(data->accel_x, 0.0f)
                 && FLOAT_EQ(data->accel_y, 0.0f)
                 && FLOAT_EQ(data->accel_z, 0.0f)
                 && FLOAT_EQ(data->gyro_x,  0.0f)
                 && FLOAT_EQ(data->gyro_y,  0.0f)
                 && FLOAT_EQ(data->gyro_z,  0.0f);

    bool stale = FLOAT_EQ(data->accel_x, health->last_mpu_ax)
              && FLOAT_EQ(data->accel_y, health->last_mpu_ay)
              && FLOAT_EQ(data->accel_z, health->last_mpu_az);

    if (all_zero || stale) {
        health->mpu6050_stale_count++;
        if (all_zero) {
            ESP_LOGW(TAG, "MPU6050: leitura zerada — leitura %d/%d",
                     health->mpu6050_stale_count, HEALTH_STALE_THRESHOLD);
        } else {
            ESP_LOGW(TAG, "MPU6050: dados repetidos — leitura %d/%d",
                     health->mpu6050_stale_count, HEALTH_STALE_THRESHOLD);
        }
    } else {
        health->mpu6050_stale_count = 0;
    }

    health->last_mpu_ax = data->accel_x;
    health->last_mpu_ay = data->accel_y;
    health->last_mpu_az = data->accel_z;

    if (health->mpu6050_stale_count >= HEALTH_STALE_THRESHOLD) {
        ESP_LOGW(TAG, "MPU6050: reinicializando sensor (reset #%lu)...",
                 (unsigned long)++health->mpu6050_resets);
        health->mpu6050_stale_count = 0;

        i2c_master_bus_rm_device(sensor->dev);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (mpu6050_init(sensor, bus) == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050: reinicializado com sucesso");
            return true;
        }
        ESP_LOGE(TAG, "MPU6050: falha na reinicializacao — verificar conexao fisica");
    }

    return false;
}

/* ---------- BH1750 ---------- */

bool sensor_health_check_bh1750(sensor_health_t *health,
                                 bh1750_handle_t *sensor,
                                 float lux,
                                 i2c_master_bus_handle_t bus)
{
    bool stale = FLOAT_EQ(lux, health->last_lux);

    if (stale) {
        health->bh1750_stale_count++;
        ESP_LOGW(TAG, "BH1750: lux repetido (%.1f) — leitura %d/%d",
                 lux, health->bh1750_stale_count, HEALTH_STALE_THRESHOLD);
    } else {
        health->bh1750_stale_count = 0;
    }

    health->last_lux = lux;

    if (health->bh1750_stale_count >= HEALTH_STALE_THRESHOLD) {
        ESP_LOGW(TAG, "BH1750: reinicializando sensor (reset #%lu)...",
                 (unsigned long)++health->bh1750_resets);
        health->bh1750_stale_count = 0;

        i2c_master_bus_rm_device(sensor->dev);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (bh1750_init(sensor, bus) == ESP_OK) {
            ESP_LOGI(TAG, "BH1750: reinicializado com sucesso");
            return true;
        }
        ESP_LOGE(TAG, "BH1750: falha na reinicializacao");
    }

    return false;
}