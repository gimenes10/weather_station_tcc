/**
 * @file lora_sx1262.h
 * @brief Driver do transceiver LoRa SX1262 via SPI para o Heltec WiFi LoRa 32 V3.
 *
 * Implementa inicialização, configuração e transmissão de pacotes LoRa.
 * O SX1262 é controlado via SPI (SPI2_HOST) com pinos de controle GPIO.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    void *spi_dev;  /* spi_device_handle_t (opaco para não expor SPI no header) */
} lora_sx1262_handle_t;

/**
 * @brief Inicializa o SX1262: reset, configura SPI, define parâmetros RF.
 * @param handle  Handle do driver
 * @return ESP_OK em sucesso
 */
esp_err_t lora_sx1262_init(lora_sx1262_handle_t *handle);

/**
 * @brief Transmite um pacote LoRa e aguarda conclusão.
 * @param handle   Handle do driver
 * @param data     Buffer com os dados a transmitir
 * @param len      Tamanho dos dados (max 255 bytes)
 * @return ESP_OK em sucesso, ESP_ERR_TIMEOUT se TX não completou
 */
esp_err_t lora_sx1262_transmit(lora_sx1262_handle_t *handle,
                               const uint8_t *data, uint8_t len);
