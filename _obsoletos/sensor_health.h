/**
 * @file sensor_health.h
 * @brief Recuperação individual dos sensores I2C baseada em falha de leitura.
 *
 * Só reage a erro de comunicação (código de retorno da leitura) — valores
 * repetidos são comportamento esperado (noite, tempo estável) e não indicam
 * falha. A falha simultânea de todos os sensores é tratada pela escada de
 * recuperação do barramento no weather_task.
 *
 * - Sensor online com HEALTH_FAIL_THRESHOLD leituras seguidas com erro:
 *   remove o device e reinicializa.
 * - Sensor offline (dev == NULL): tenta reinicializar a cada
 *   HEALTH_RETRY_CYCLES ciclos (hot-plug).
 */
#pragma once

#include "bme280.h"
#include "mpu6050.h"
#include "bh1750.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/* Leituras seguidas com erro para reinicializar o sensor */
#define HEALTH_FAIL_THRESHOLD   3

/* Ciclos entre tentativas de reinit de um sensor offline (12 x 5s = 60s) */
#define HEALTH_RETRY_CYCLES     12

/* Estado de recuperação de um sensor */
typedef struct {
    uint8_t  fail_count;       /* Leituras seguidas com erro */
    uint8_t  offline_cycles;   /* Ciclos desde a última tentativa (offline) */
    uint32_t resets;           /* Reinicializações por falha de leitura */
} sensor_health_entry_t;

typedef struct {
    sensor_health_entry_t bme280;
    sensor_health_entry_t mpu6050;
    sensor_health_entry_t bh1750;
} sensor_health_t;

void sensor_health_init(sensor_health_t *health);

/**
 * @param read_ok  true se a leitura deste ciclo retornou ESP_OK
 * @return true se o sensor foi reinicializado com sucesso neste ciclo
 */
bool sensor_health_check_bme280(sensor_health_t *health, bme280_handle_t *sensor,
                                bool read_ok, i2c_master_bus_handle_t bus);

bool sensor_health_check_mpu6050(sensor_health_t *health, mpu6050_handle_t *sensor,
                                 bool read_ok, i2c_master_bus_handle_t bus);

bool sensor_health_check_bh1750(sensor_health_t *health, bh1750_handle_t *sensor,
                                bool read_ok, i2c_master_bus_handle_t bus);
