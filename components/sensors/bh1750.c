#include "bh1750.h"
#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"
#include "esp_timer.h"

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
static uint8_t                 s_addr;

static esp_err_t send(uint8_t cmd)
{
    return i2c_master_transmit(s_dev, &cmd, 1, I2C_TIMEOUT_MS);
}

/*
 * Solta o sensor depois de um erro. O endereço vem do pino ADDR, lido pelo chip
 * o tempo todo: com o ADDR solto ou com mau contato ele pula entre 0x23 e 0x5C.
 * Com s_dev em NULL, a próxima chamada procura de novo nos dois endereços.
 */
static void drop(const char *where, esp_err_t err)
{
    ESP_LOGW(TAG, "Parou de responder em 0x%02X (%s, %s): procurando de novo nos dois enderecos. "
             "Se repetir, confira a solda do pino ADDR", s_addr, where, esp_err_to_name(err));
    i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
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
    s_addr = addr;
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(s_dev);
err_add:
    s_dev = NULL;
    return ret;
}

esp_err_t bh1750_init(i2c_master_bus_handle_t bus)
{
    /* Endereço principal e alternativo no pin_config.h (0x23 = ADDR no GND, 0x5C = ADDR alto) */
    static const uint8_t addrs[] = { BH1750_I2C_ADDR, BH1750_I2C_ADDR_ALT };
    esp_err_t ret = ESP_FAIL;

    for (size_t i = 0; i < sizeof(addrs); i++) {
        const int64_t t0 = esp_timer_get_time();

        ret = try_addr(bus, addrs[i]);
        if (ret == ESP_OK) {
            break;
        }
        /* ~1 ms = NACK (ninguém no endereço); ~100 ms = barramento sem clock */
        ESP_LOGW(TAG, "0x%02X sem resposta: %s em %lld ms", addrs[i], esp_err_to_name(ret),
                 (long long)((esp_timer_get_time() - t0) / 1000));
    }
    return ret;
}

void bh1750_deinit(void)
{
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
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
            drop("disparo", ret);
            ret = ESP_ERR_INVALID_STATE;   /* Faz o station.c reinicializar já neste ciclo */
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
        drop("leitura", ret);
        return ret;
    }
    *lux = (float)((raw[0] << 8) | raw[1]) / 1.2f * ((float)MTREG_DEFAULT / MTREG) / DOME_TRANSMITTANCE;
    return ESP_OK;
}
