#include "bh1750.h"
#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"

static const char *TAG = "bh1750";

#define I2C_TIMEOUT_MS      100

#define CMD_POWER_ON        0x01
#define CMD_ONE_TIME_H      0x20
#define CMD_MTREG_HI        0x40    /* 01000_MT[7:5] */
#define CMD_MTREG_LO        0x60    /* 011_MT[4:0]   */

#define MTREG_DEFAULT       69
#define MTREG               31
/* TODO calibrar: razão lux sem cúpula / lux com cúpula, medida lado a lado */
#define DOME_TRANSMITTANCE  1.0f

static i2c_master_dev_handle_t s_dev;

static esp_err_t send(uint8_t cmd)
{
    return i2c_master_transmit(s_dev, &cmd, 1, I2C_TIMEOUT_MS);
}

/* Adiciona o sensor no endereço e confirma com POWER_ON */
static esp_err_t try_addr(i2c_master_bus_handle_t bus, uint8_t addr)
{
    esp_err_t ret = i2c_bus_add(bus, addr, &s_dev);

    if (ret != ESP_OK) {
        goto err_add;
    }
    ret = send(CMD_POWER_ON);
    if (ret != ESP_OK) {
        goto err_dev;
    }
    ESP_LOGI(TAG, "BH1750 em 0x%02X", addr);
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(s_dev);
err_add:
    s_dev = NULL;
    return ret;
}

esp_err_t bh1750_init(i2c_master_bus_handle_t bus)
{
    /* 0x23 com ADDR no GND; 0x5C se o pino ADDR estiver em nível alto */
    esp_err_t ret = try_addr(bus, BH1750_I2C_ADDR);

    if (ret != ESP_OK) {
        ret = try_addr(bus, BH1750_I2C_ADDR_ALT);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Sem resposta em 0x%02X nem 0x%02X", BH1750_I2C_ADDR, BH1750_I2C_ADDR_ALT);
    }
    return ret;
}

esp_err_t bh1750_trigger(void)
{
    static const uint8_t seq[] = {
        CMD_POWER_ON,
        CMD_MTREG_HI | (MTREG >> 5),
        CMD_MTREG_LO | (MTREG & 0x1F),
        CMD_ONE_TIME_H,
    };
    esp_err_t ret = ESP_ERR_INVALID_STATE;

    if (s_dev == NULL) {
        goto err;
    }
    for (size_t i = 0; i < sizeof(seq); i++) {
        ret = send(seq[i]);
        if (ret != ESP_OK) {
            goto err;
        }
    }
    return ESP_OK;

err:
    return ret;
}

esp_err_t bh1750_read(float *lux)
{
    uint8_t   raw[2];
    esp_err_t ret;

    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    ret = i2c_master_receive(s_dev, raw, sizeof(raw), I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    *lux = (float)((raw[0] << 8) | raw[1]) / 1.2f * ((float)MTREG_DEFAULT / MTREG) / DOME_TRANSMITTANCE;
    return ESP_OK;
}
