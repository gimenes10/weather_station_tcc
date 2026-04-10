/**
 * @file main.c
 * @brief Ponto de entrada da estação meteorológica IoT.
 *
 * Projeto: TCC — Estação Meteorológica IoT com comunicação LoRa
 * Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * Framework: ESP-IDF v6.0
 *
 * Sensores:
 *   I2C (GPIO47 SDA / GPIO48 SCL):
 *     - BME280   (0x76) — Temperatura, umidade, pressão
 *     - MPU-6050 (0x68) — Acelerômetro + giroscópio
 *     - BH1750   (0x23) — Luminosidade
 *   ADC:
 *     - MQ-7  (GPIO1)  — Monóxido de carbono (CO)
 *     - MQ-2  (GPIO2)  — Gases combustíveis (GLP, metano, H₂)
 *     - FC-22 (GPIO3)  — Sensor de chuva (VCC via GPIO7)
 *   OLED:
 *     - SSD1306 128x64 (GPIO17 SDA / GPIO18 SCL / GPIO21 RST)
 */

#include "weather_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

#define WEATHER_TASK_STACK_SIZE  8192
#define WEATHER_TASK_PRIORITY    5

void app_main(void)
{
    esp_err_t ret = weather_station_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicializacao: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Verifique as conexoes fisicas e reinicie.");
        return;
    }

    /* Cria task de leitura dos sensores */
    BaseType_t xret = xTaskCreate(
        weather_task,
        "weather_task",
        WEATHER_TASK_STACK_SIZE,
        NULL,
        WEATHER_TASK_PRIORITY,
        NULL
    );

    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar weather_task");
        return;
    }

    ESP_LOGI(TAG, "weather_task criada — monitoramento ativo");
}
