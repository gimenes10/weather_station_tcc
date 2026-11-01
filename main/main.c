/**
 * @file main.c
 * @brief Nó sensor da estação meteorológica (TCC — UNIFESP ICT).
 *
 * Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262), ESP-IDF v6.1, alimentado
 * pelo USB-C. Lê BME680, BH1750, GP2Y1014, FC-22, anemômetro e
 * pluviômetro e transmite um pacote LoRa por minuto.
 */
#include "station.h"
#include "esp_log.h"

void app_main(void)
{
    esp_err_t ret = station_init();

    if (ret != ESP_OK) {
        ESP_LOGE("main", "Falha na inicializacao: %s", esp_err_to_name(ret));
        return;
    }
    station_run();
}
