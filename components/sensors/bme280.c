#include "bme280.h"
#include "pin_config.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bme280";
#define I2C_TIMEOUT_MS  100

/* ---------- Registradores ---------- */
#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA_START   0xF7
#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_26     0xE1
#define BME280_CHIP_ID          0x60

/* ---------- Helpers I2C ---------- */

static esp_err_t reg_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT_MS);
}

static esp_err_t reg_read(i2c_master_dev_handle_t dev, uint8_t reg,
                          uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

/* ---------- Leitura dos coeficientes de calibração ---------- */

static esp_err_t load_calibration(bme280_handle_t *s)
{
    esp_err_t ret;
    uint8_t buf[26];

    /* Bloco 0x88..0xA1 (26 bytes) — T e P */
    ret = reg_read(s->dev, BME280_REG_CALIB_00, buf, 26);
    if (ret != ESP_OK) { return ret; }

    s->dig_T1 = (uint16_t)(buf[1]  << 8) | buf[0];
    s->dig_T2 = (int16_t) (buf[3]  << 8) | buf[2];
    s->dig_T3 = (int16_t) (buf[5]  << 8) | buf[4];
    s->dig_P1 = (uint16_t)(buf[7]  << 8) | buf[6];
    s->dig_P2 = (int16_t) (buf[9]  << 8) | buf[8];
    s->dig_P3 = (int16_t) (buf[11] << 8) | buf[10];
    s->dig_P4 = (int16_t) (buf[13] << 8) | buf[12];
    s->dig_P5 = (int16_t) (buf[15] << 8) | buf[14];
    s->dig_P6 = (int16_t) (buf[17] << 8) | buf[16];
    s->dig_P7 = (int16_t) (buf[19] << 8) | buf[18];
    s->dig_P8 = (int16_t) (buf[21] << 8) | buf[20];
    s->dig_P9 = (int16_t) (buf[23] << 8) | buf[22];
    s->dig_H1 = buf[25];

    /* Bloco 0xE1..0xE7 (7 bytes) — H */
    uint8_t hbuf[7];
    ret = reg_read(s->dev, BME280_REG_CALIB_26, hbuf, 7);
    if (ret != ESP_OK) { return ret; }

    s->dig_H2 = (int16_t)(hbuf[1] << 8) | hbuf[0];
    s->dig_H3 = hbuf[2];
    s->dig_H4 = (int16_t)((int16_t)hbuf[3] << 4) | (hbuf[4] & 0x0F);
    s->dig_H5 = (int16_t)((int16_t)hbuf[5] << 4) | (hbuf[4] >> 4);
    s->dig_H6 = (int8_t)hbuf[6];

    return ESP_OK;
}

/* ---------- Init ---------- */

esp_err_t bme280_init(bme280_handle_t *sensor, i2c_master_bus_handle_t bus)
{
    esp_err_t ret;
    uint8_t chip_id = 0;

    /* Adiciona o dispositivo ao barramento */
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME280_I2C_ADDR,
        .scl_speed_hz    = SENSOR_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus, &cfg, &sensor->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo: %s", esp_err_to_name(ret));
        goto err_add;
    }

    /* Verifica chip ID */
    ret = reg_read(sensor->dev, BME280_REG_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura do chip ID: %s", esp_err_to_name(ret));
        goto err_id;
    }
    if (chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Chip ID inesperado: 0x%02X (esperado 0x%02X)", chip_id, BME280_CHIP_ID);
        ret = ESP_ERR_INVALID_RESPONSE;
        goto err_id;
    }

    /* Soft reset */
    ret = reg_write(sensor->dev, BME280_REG_RESET, 0xB6);
    if (ret != ESP_OK) { goto err_id; }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Carrega calibração */
    ret = load_calibration(sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler calibração: %s", esp_err_to_name(ret));
        goto err_id;
    }

    /* Config: standby 1s, filtro coef. 4 */
    ret = reg_write(sensor->dev, BME280_REG_CONFIG, (0x04 << 5) | (0x02 << 2));
    if (ret != ESP_OK) { goto err_id; }

    /* Oversampling: umidade x1 */
    ret = reg_write(sensor->dev, BME280_REG_CTRL_HUM, 0x01);
    if (ret != ESP_OK) { goto err_id; }

    /* Oversampling: temp x2, press x16, modo normal */
    ret = reg_write(sensor->dev, BME280_REG_CTRL_MEAS, (0x02 << 5) | (0x05 << 2) | 0x03);
    if (ret != ESP_OK) { goto err_id; }

    ESP_LOGI(TAG, "BME280 inicializado (ID=0x%02X)", chip_id);
    return ESP_OK;

err_id:
    i2c_master_bus_rm_device(sensor->dev);
err_add:
    return ret;
}

/* ---------- Compensação (algoritmo do datasheet Bosch) ---------- */

static float compensate_temperature(bme280_handle_t *s, int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s->dig_T1 << 1))) * (int32_t)s->dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)s->dig_T1) *
                      ((adc_T >> 4) - (int32_t)s->dig_T1)) >> 12) * (int32_t)s->dig_T3) >> 14;
    s->t_fine = var1 + var2;
    return (float)((s->t_fine * 5 + 128) >> 8) / 100.0f;
}

static float compensate_pressure(bme280_handle_t *s, int32_t adc_P)
{
    int64_t var1 = (int64_t)s->t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s->dig_P6;
    var2 = var2 + ((var1 * (int64_t)s->dig_P5) << 17);
    var2 = var2 + (((int64_t)s->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s->dig_P3) >> 8) + ((var1 * (int64_t)s->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * (int64_t)s->dig_P1 >> 33;
    if (var1 == 0) { return 0.0f; }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)s->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)s->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)s->dig_P7 << 4);
    return (float)p / 25600.0f;
}

static float compensate_humidity(bme280_handle_t *s, int32_t adc_H)
{
    int32_t v = s->t_fine - 76800;
    v = ((((adc_H << 14) - ((int32_t)s->dig_H4 << 20) - ((int32_t)s->dig_H5 * v)) + 16384) >> 15) *
        (((((((v * (int32_t)s->dig_H6) >> 10) * (((v * (int32_t)s->dig_H3) >> 11) + 32768)) >> 10) +
           2097152) * (int32_t)s->dig_H2 + 8192) >> 14);
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)s->dig_H1) >> 4);
    v = (v < 0) ? 0 : v;
    v = (v > 419430400) ? 419430400 : v;
    return (float)(v >> 12) / 1024.0f;
}

/* ---------- Leitura ---------- */

esp_err_t bme280_read(bme280_handle_t *sensor, bme280_data_t *data)
{
    esp_err_t ret;
    uint8_t raw[8];

    ret = reg_read(sensor->dev, BME280_REG_DATA_START, raw, 8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura de dados: %s", esp_err_to_name(ret));
        goto err_read;
    }

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] <<  8) |  (int32_t)raw[7];

    /* Temperatura deve ser calculada primeiro (define t_fine) */
    data->temperature_c = compensate_temperature(sensor, adc_T);
    data->pressure_hpa  = compensate_pressure(sensor, adc_P);
    data->humidity_pct  = compensate_humidity(sensor, adc_H);

    return ESP_OK;

err_read:
    return ret;
}
