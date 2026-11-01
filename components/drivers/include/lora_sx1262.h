/**
 * @file lora_sx1262.h
 * @brief Driver do transceiver LoRa SX1262 via SPI para o Heltec WiFi LoRa 32 V3.
 *
 * Implementa inicialização, configuração e transmissão de pacotes LoRa.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    void *spi_dev;  /* spi_device_handle_t (opaco para não expor SPI no header) */
} lora_sx1262_handle_t;

/**
 * @brief Reset, SPI e parâmetros RF.
 */
esp_err_t lora_sx1262_init(lora_sx1262_handle_t *handle);

/**
 * @brief Transmite um pacote LoRa e aguarda TX_DONE.
 * @return ESP_ERR_TIMEOUT se o TX não completou
 */
esp_err_t lora_sx1262_transmit(lora_sx1262_handle_t *handle,
                               const uint8_t *data, uint8_t len);
