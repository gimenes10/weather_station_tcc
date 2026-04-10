#include "bh1750.h"
#include "pin_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bh1750";
#define I2C_TIMEOUT_MS  200

/* Comandos BH1750 */
#define BH1750_CMD_POWER_ON       0x01
#define BH1750_CMD_RESET          0x07
#define BH1750_CMD_ONE_TIME_H_RES 0x20  /* One-time, H-resolution (1 lux, ~120ms) */

static esp_err_t send_cmd(i2c_master_dev_handle_t dev, uint8_t cmd)
{
    return i2c_master_transmit(dev, &cmd, 1, I2C_TIMEOUT_MS);
}

esp_err_t bh1750_init(bh1750_handle_t *sensor, i2c_master_bus_handle_t bus)
{
    esp_err_t ret;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BH1750_I2C_ADDR,
        .scl_speed_hz    = SENSOR_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus, &cfg, &sensor->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo: %s", esp_err_to_name(ret));
        goto err_add;
    }

    /* Power on + reset */
    ret = send_cmd(sensor->dev, BH1750_CMD_POWER_ON);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no power on: %s", esp_err_to_name(ret));
        goto err_cmd;
    }
    ret = send_cmd(sensor->dev, BH1750_CMD_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no reset: %s", esp_err_to_name(ret));
        goto err_cmd;
    }

    ESP_LOGI(TAG, "BH1750 inicializado (addr=0x%02X)", BH1750_I2C_ADDR);
    return ESP_OK;

err_cmd:
    i2c_master_bus_rm_device(sensor->dev);
err_add:
    return ret;
}

esp_err_t bh1750_read(bh1750_handle_t *sensor, float *lux)
{
    esp_err_t ret;
    uint8_t raw[2];

    /* Dispara medição one-time */
    ret = send_cmd(sensor->dev, BH1750_CMD_ONE_TIME_H_RES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao disparar medição: %s", esp_err_to_name(ret));
        goto err_read;
    }

    /* Aguarda conversão (~120ms no modo H-resolution) */
    vTaskDelay(pdMS_TO_TICKS(180));

    /* Lê 2 bytes do resultado */
    ret = i2c_master_receive(sensor->dev, raw, 2, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura: %s", esp_err_to_name(ret));
        goto err_read;
    }

    /* Converte: valor bruto / 1.2 = lux */
    *lux = (float)((raw[0] << 8) | raw[1]) / 1.2f;
    return ESP_OK;

err_read:
    return ret;
}
