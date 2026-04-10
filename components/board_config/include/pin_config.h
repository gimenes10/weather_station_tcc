/**
 * @file pin_config.h
 * @brief Definição dos pinos do Heltec WiFi LoRa 32 V3 para a estação meteorológica.
 *
 * Mapeamento baseado na documentação oficial da Heltec:
 * - I2C sensores: GPIO 47 (SDA), GPIO 48 (SCL) — I2C_NUM_1
 * - I2C OLED:     GPIO 17 (SDA), GPIO 18 (SCL) — I2C_NUM_0
 * - ADC:          GPIO 1..3 — ADC1_CH0..CH2
 * - Rain power:   GPIO 7 — liga/desliga alimentação do sensor de chuva
 */
#pragma once

/* ---------- I2C — Barramento dos sensores (I2C_NUM_1) ---------- */
#define SENSOR_I2C_PORT     I2C_NUM_1
#define SENSOR_I2C_SDA      GPIO_NUM_47
#define SENSOR_I2C_SCL      GPIO_NUM_48
#define SENSOR_I2C_FREQ_HZ  100000      /* 100 kHz — Standard mode */

/* ---------- I2C — OLED SSD1306 (I2C_NUM_0) ---------- */
#define OLED_I2C_PORT       I2C_NUM_0
#define OLED_I2C_SDA        GPIO_NUM_17
#define OLED_I2C_SCL        GPIO_NUM_18
#define OLED_I2C_FREQ_HZ    400000      /* 400 kHz — Fast mode */
#define OLED_RST_PIN        GPIO_NUM_21
#define OLED_I2C_ADDR       0x3C

/* ---------- Endereços I2C dos sensores ---------- */
#define BME280_I2C_ADDR     0x76
#define MPU6050_I2C_ADDR    0x68
#define BH1750_I2C_ADDR     0x23

/* ---------- ADC — Sensores analógicos ---------- */
#define MQ7_ADC_CHANNEL     ADC_CHANNEL_0   /* GPIO 1 */
#define MQ2_ADC_CHANNEL     ADC_CHANNEL_1   /* GPIO 2 */
#define RAIN_ADC_CHANNEL    ADC_CHANNEL_2   /* GPIO 3 */

/* ---------- GPIO — Controle de alimentação ---------- */
#define RAIN_POWER_PIN      GPIO_NUM_7      /* Liga/desliga VCC do FC-22 */
