/**
 * @file i2c_bus.h
 * @brief Barramento I2C dos sensores e acesso a registradores.
 *
 * Um único barramento (SDA GPIO41, SCL GPIO40, 10 kHz) com BME680 e BH1750.
 * Pull-ups: os dos módulos, mais os internos do ESP32 (~45 kΩ).
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/** @brief Cria o barramento dos sensores. */
esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus);

/**
 * @brief Varre 0x08–0x77 e loga os endereços que responderam.
 * @return Quantidade de dispositivos encontrados
 */
int i2c_bus_scan(i2c_master_bus_handle_t bus);

/** @brief Libera um barramento travado (9 pulsos em SCL + reset do controlador). */
esp_err_t i2c_bus_reset(i2c_master_bus_handle_t bus);

/**
 * @brief Recuperação completa: apaga o driver, libera as linhas por GPIO
 *        (pulsos em SCL até o SDA soltar + STOP) e cria o barramento de novo.
 *        Os dispositivos registrados precisam ser removidos antes.
 * @param bus Handle atual; recebe o novo
 */
esp_err_t i2c_bus_recover(i2c_master_bus_handle_t *bus);

/** @brief Adiciona um dispositivo de 7 bits na frequência do barramento. */
esp_err_t i2c_bus_add(i2c_master_bus_handle_t bus, uint8_t addr, i2c_master_dev_handle_t *dev);

/** @brief Escreve um byte num registrador de 8 bits. */
esp_err_t i2c_reg_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);

/** @brief Lê `len` bytes a partir de `reg`. */
esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
