#include "mpu6050.h"
#include "pin_config.h"
#include "esp_log.h"

static const char *TAG = "mpu6050";
#define I2C_TIMEOUT_MS  100

/* Registradores */
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_WHO_AM_I      0x75
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_WHO_AM_I_VAL      0x68

/* Escalas: Accel ±2g (16384 LSB/g), Gyro ±250°/s (131 LSB/°/s) */
#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE   131.0f

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

esp_err_t mpu6050_init(mpu6050_handle_t *sensor, i2c_master_bus_handle_t bus)
{
    esp_err_t ret;
    uint8_t who = 0;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_I2C_ADDR,
        .scl_speed_hz    = SENSOR_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus, &cfg, &sensor->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar dispositivo: %s", esp_err_to_name(ret));
        goto err_add;
    }

    /* Verifica WHO_AM_I */
    ret = reg_read(sensor->dev, MPU_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura WHO_AM_I: %s", esp_err_to_name(ret));
        goto err_cfg;
    }
    if (who != MPU_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I inesperado: 0x%02X (esperado 0x%02X)", who, MPU_WHO_AM_I_VAL);
        ret = ESP_ERR_INVALID_RESPONSE;
        goto err_cfg;
    }

    /* Acorda o sensor (sai do sleep) — clock PLL com referência gyro X */
    ret = reg_write(sensor->dev, MPU_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) { goto err_cfg; }
    vTaskDelay(pdMS_TO_TICKS(30));

    /* Sample rate divider: 1 kHz / (1+9) = 100 Hz */
    ret = reg_write(sensor->dev, MPU_REG_SMPLRT_DIV, 9);
    if (ret != ESP_OK) { goto err_cfg; }

    /* DLPF config: banda 44Hz accel, 42Hz gyro */
    ret = reg_write(sensor->dev, MPU_REG_CONFIG, 0x03);
    if (ret != ESP_OK) { goto err_cfg; }

    /* Accel: ±2g | Gyro: ±250°/s */
    ret = reg_write(sensor->dev, MPU_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) { goto err_cfg; }
    ret = reg_write(sensor->dev, MPU_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) { goto err_cfg; }

    ESP_LOGI(TAG, "MPU-6050 inicializado (WHO_AM_I=0x%02X)", who);
    return ESP_OK;

err_cfg:
    i2c_master_bus_rm_device(sensor->dev);
err_add:
    return ret;
}

esp_err_t mpu6050_read(mpu6050_handle_t *sensor, mpu6050_data_t *data)
{
    esp_err_t ret;
    uint8_t raw[14];

    /* Lê 14 bytes: ACCEL(6) + TEMP(2) + GYRO(6) */
    ret = reg_read(sensor->dev, MPU_REG_ACCEL_XOUT_H, raw, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na leitura: %s", esp_err_to_name(ret));
        goto err_read;
    }

    data->accel_x = (float)((int16_t)(raw[0]  << 8 | raw[1]))  / ACCEL_SCALE;
    data->accel_y = (float)((int16_t)(raw[2]  << 8 | raw[3]))  / ACCEL_SCALE;
    data->accel_z = (float)((int16_t)(raw[4]  << 8 | raw[5]))  / ACCEL_SCALE;
    data->temperature_c = ((float)((int16_t)(raw[6] << 8 | raw[7])) / 340.0f) + 36.53f;
    data->gyro_x  = (float)((int16_t)(raw[8]  << 8 | raw[9]))  / GYRO_SCALE;
    data->gyro_y  = (float)((int16_t)(raw[10] << 8 | raw[11])) / GYRO_SCALE;
    data->gyro_z  = (float)((int16_t)(raw[12] << 8 | raw[13])) / GYRO_SCALE;

    return ESP_OK;

err_read:
    return ret;
}
