#include "ina219.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define REG_CONFIG      0x00
#define REG_SHUNT       0x01
#define REG_BUS         0x02

#define CFG_BRNG_16V    (0U << 13)
#define CFG_PG(x)       ((uint16_t)(x) << 11)
#define CFG_BADC_12x16  (0xCU << 7)     /* 12 bits, média de 16: 8,51 ms */
#define CFG_SADC_12x16  (0xCU << 3)
#define MODE_POWERDOWN  0x0U
#define MODE_TRIG_SB    0x3U            /* shunt + barramento, disparo único */
#define MODE_MSK        0x7U

#define BUS_CNVR        (1U << 1)
#define BUS_LSB_MV      4

/* TODO conferir: módulos comuns trazem shunt R100 (0,1 Ω) */
#define SHUNT_OHM       0.1f
#define SHUNT_LSB_MV    0.01f           /* 10 µV */

#define POLL_MS         2
#define POLL_TIMEOUT_MS 50

esp_err_t ina219_init(ina219_t *s, i2c_master_bus_handle_t bus, uint8_t addr, ina219_pga_t pga)
{
    uint16_t  rd;
    esp_err_t ret = i2c_bus_add(bus, addr, &s->dev);

    if (ret != ESP_OK) {
        goto err_add;
    }

    s->cfg = CFG_BRNG_16V | CFG_PG(pga) | CFG_BADC_12x16 | CFG_SADC_12x16;
    ret = i2c_reg_write16(s->dev, REG_CONFIG, s->cfg | MODE_POWERDOWN);
    if (ret != ESP_OK) {
        goto err_dev;
    }
    ret = i2c_reg_read16(s->dev, REG_CONFIG, &rd);
    if (ret != ESP_OK) {
        goto err_dev;
    }
    if (rd != (s->cfg | MODE_POWERDOWN)) {
        ret = ESP_ERR_INVALID_RESPONSE;
        goto err_dev;
    }
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(s->dev);
err_add:
    s->dev = NULL;
    return ret;
}

esp_err_t ina219_trigger(ina219_t *s)
{
    if (s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_reg_write16(s->dev, REG_CONFIG, s->cfg | MODE_TRIG_SB);
}

esp_err_t ina219_read(ina219_t *s, int *bus_mv, float *ma)
{
    uint16_t  bus;
    uint16_t  sh;
    uint32_t  waited = 0;
    esp_err_t ret;

    if (s->dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    for (;;) {
        ret = i2c_reg_read16(s->dev, REG_BUS, &bus);
        if (ret != ESP_OK) { goto err; }
        if (bus & BUS_CNVR) { break; }
        if (waited >= POLL_TIMEOUT_MS) {
            ret = ESP_ERR_TIMEOUT;
            goto err;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        waited += POLL_MS;
    }

    ret = i2c_reg_read16(s->dev, REG_SHUNT, &sh);
    if (ret != ESP_OK) { goto err; }

    *bus_mv = (int)(bus >> 3) * BUS_LSB_MV;
    *ma     = (float)(int16_t)sh * SHUNT_LSB_MV / SHUNT_OHM;

err:
    /* Power-down mesmo após falha: no modo ativo o INA219 consome ~1 mA */
    (void)i2c_reg_write16(s->dev, REG_CONFIG, s->cfg | MODE_POWERDOWN);
    return ret;
}
