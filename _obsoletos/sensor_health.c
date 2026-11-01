#include "sensor_health.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "health";

typedef esp_err_t (*sensor_init_fn_t)(void *sensor, i2c_master_bus_handle_t bus);

/* Adaptadores: assinatura comum para o check genérico */
static esp_err_t bme280_init_cb(void *s, i2c_master_bus_handle_t bus)  { return bme280_init(s, bus); }
static esp_err_t mpu6050_init_cb(void *s, i2c_master_bus_handle_t bus) { return mpu6050_init(s, bus); }
static esp_err_t bh1750_init_cb(void *s, i2c_master_bus_handle_t bus)  { return bh1750_init(s, bus); }

void sensor_health_init(sensor_health_t *health)
{
    memset(health, 0, sizeof(*health));
}

/**
 * @brief Lógica comum: reinit após falhas seguidas de leitura ou, se
 *        offline, tentativa periódica (hot-plug).
 *
 * @param dev  Ponteiro para o campo `dev` do handle do sensor
 */
static bool health_check(const char *name, sensor_health_entry_t *h,
                         i2c_master_dev_handle_t *dev, void *sensor,
                         sensor_init_fn_t init, bool read_ok,
                         i2c_master_bus_handle_t bus)
{
    /* Offline: tenta de novo a cada HEALTH_RETRY_CYCLES */
    if (*dev == NULL) {
        if (++h->offline_cycles < HEALTH_RETRY_CYCLES) {
            return false;
        }
        h->offline_cycles = 0;
        ESP_LOGI(TAG, "%s: offline — tentando reinicializar...", name);
        goto reinit;
    }

    if (read_ok) {
        h->fail_count = 0;
        return false;
    }

    if (++h->fail_count < HEALTH_FAIL_THRESHOLD) {
        ESP_LOGW(TAG, "%s: falha de leitura %u/%d", name, h->fail_count, HEALTH_FAIL_THRESHOLD);
        return false;
    }

    h->fail_count = 0;
    ESP_LOGW(TAG, "%s: reinicializando (reset #%lu)...", name, (unsigned long)++h->resets);
    i2c_master_bus_rm_device(*dev);
    *dev = NULL;
    vTaskDelay(pdMS_TO_TICKS(50));

reinit:
    if (init(sensor, bus) == ESP_OK) {
        ESP_LOGI(TAG, "%s: reinicializado com sucesso", name);
        h->offline_cycles = 0;
        return true;
    }
    ESP_LOGE(TAG, "%s: falha na reinicializacao — nova tentativa em %d ciclos",
             name, HEALTH_RETRY_CYCLES);
    return false;
}

bool sensor_health_check_bme280(sensor_health_t *health, bme280_handle_t *sensor,
                                bool read_ok, i2c_master_bus_handle_t bus)
{
    return health_check("BME280", &health->bme280, &sensor->dev, sensor,
                        bme280_init_cb, read_ok, bus);
}

bool sensor_health_check_mpu6050(sensor_health_t *health, mpu6050_handle_t *sensor,
                                 bool read_ok, i2c_master_bus_handle_t bus)
{
    return health_check("MPU6050", &health->mpu6050, &sensor->dev, sensor,
                        mpu6050_init_cb, read_ok, bus);
}

bool sensor_health_check_bh1750(sensor_health_t *health, bh1750_handle_t *sensor,
                                bool read_ok, i2c_master_bus_handle_t bus)
{
    return health_check("BH1750", &health->bh1750, &sensor->dev, sensor,
                        bh1750_init_cb, read_ok, bus);
}
