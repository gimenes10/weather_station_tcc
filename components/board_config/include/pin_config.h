/**
 * @file pin_config.h
 * @brief Mapa de pinos do nó sensor (Heltec WiFi LoRa 32 V3) e parâmetros fixos.
 *
 * Segue o diagrama "Estação meteorológica: onde ligar cada fio".
 *
 * | GPIO  | Função                                   | Observação                       |
 * |-------|------------------------------------------|----------------------------------|
 * | 41/40 | SDA/SCL I2C (BME680, BH1750)             | 10 kHz, pull-ups nos módulos     |
 * | 7     | Vo do GP2Y pelo divisor 12k/20k (ADC1_6) | Atenuação 12 dB                  |
 * | 6     | LED do GP2Y (base de Q2 pelo R3 1 kΩ)   | ALTO acende                      |
 * | 38    | Alimentação do FC-22 pelo JB (opcional) | Sempre em nível alto             |
 * | 2     | AO do FC-22 (ADC1_1)                     |                                  |
 * | 4     | Reed do anemômetro (interrupção)         | Pull-up externo 100k + 10 nF     |
 * | 5     | Reed do pluviômetro (interrupção)        | Pull-up externo 100k + 10 nF     |
 *
 * Alimentação pelo USB-C: o GP2Y fica no 5V do Heltec o tempo todo.
 * GPIO42 não é usado: nesta placa a entrada dele lê sempre 0 (SCL foi para o GPIO40).
 * GPIO4 a GPIO7 chegam à fenolite num único cabo de 4 vias (conector JC).
 */
#pragma once

#include "hal/gpio_types.h"
#include "hal/adc_types.h"

/* ---------- I2C dos sensores ---------- */
#define SENSOR_I2C_PORT       I2C_NUM_0
#define SENSOR_I2C_SDA        GPIO_NUM_41
#define SENSOR_I2C_SCL        GPIO_NUM_40
#define SENSOR_I2C_FREQ_HZ    10000       /* Margem de integridade com cabos até o shield */

#define BME680_I2C_ADDR       0x77        /* SDO em nível alto (como está montado) */
#define BME680_I2C_ADDR_ALT   0x76        /* SDO no GND (tentado se 0x77 não responder) */
#define BH1750_I2C_ADDR       0x23        /* ADDR no GND (como está montado) */
#define BH1750_I2C_ADDR_ALT   0x5C        /* ADDR em nível alto (tentado se 0x23 não responder) */

/* ---------- ADC1 ---------- */
#define GP2Y_ADC_CHANNEL      ADC_CHANNEL_6   /* GPIO7 */
#define RAIN_ADC_CHANNEL      ADC_CHANNEL_1   /* GPIO2 */

/* ---------- Saídas de controle ---------- */
#define GP2Y_LED_PIN          GPIO_NUM_6
#define RAIN_POWER_PIN        GPIO_NUM_38

/* ---------- Reeds (interrupção de GPIO, reed fechado = nível 0) ---------- */
#define WIND_REED_PIN         GPIO_NUM_4
#define GAUGE_REED_PIN        GPIO_NUM_5

/* ---------- SPI — LoRa SX1262 ---------- */
#define LORA_SPI_HOST         SPI2_HOST
#define LORA_PIN_SCK          GPIO_NUM_9
#define LORA_PIN_MOSI         GPIO_NUM_10
#define LORA_PIN_MISO         GPIO_NUM_11
#define LORA_PIN_CS           GPIO_NUM_8
#define LORA_PIN_RST          GPIO_NUM_12
#define LORA_PIN_BUSY         GPIO_NUM_13
#define LORA_PIN_DIO1         GPIO_NUM_14

/* ---------- LoRa — parâmetros RF (iguais no gateway) ---------- */
#define LORA_FREQUENCY_HZ     915000000   /* ISM Brasil */
#define LORA_SF               7
#define LORA_BW               0x04        /* 125 kHz */
#define LORA_CR               0x01        /* 4/5 */
#define LORA_TX_POWER         22          /* dBm */
#define LORA_PREAMBLE_LEN     8
