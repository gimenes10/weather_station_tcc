/**
 * @file ina219.h
 * @brief INA219 em disparo único (shunt + barramento) com power-down entre leituras.
 *
 * Lê o registrador de shunt direto (LSB 10 µV) e calcula a corrente com o
 * resistor do módulo; não usa o registrador de calibração.
 *   I [mA] = Vshunt / Rshunt        V [mV] = (bus >> 3) × 4
 * ADC de 12 bits com média de 16 amostras: ~17 ms por par de conversões.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

/* Faixa do PGA (fundo de escala da tensão de shunt) */
typedef enum {
    INA219_PGA_40MV  = 0,
    INA219_PGA_80MV  = 1,
    INA219_PGA_160MV = 2,
    INA219_PGA_320MV = 3,
} ina219_pga_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint16_t                cfg;   /* Config sem os bits de modo */
} ina219_t;

/** @brief Adiciona ao barramento, grava a configuração e deixa em power-down. */
esp_err_t ina219_init(ina219_t *s, i2c_master_bus_handle_t bus, uint8_t addr, ina219_pga_t pga);

/** @brief Dispara uma conversão de shunt e barramento. */
esp_err_t ina219_trigger(ina219_t *s);

/**
 * @brief Espera a conversão, lê e volta ao power-down.
 * @param[out] bus_mv Tensão no VIN− (lado da carga)
 * @param[out] ma     Corrente de VIN+ para VIN− (com sinal)
 */
esp_err_t ina219_read(ina219_t *s, int *bus_mv, float *ma);
