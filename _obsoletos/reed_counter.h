/**
 * @file reed_counter.h
 * @brief Contador de pulsos de reed switch por interrupção de GPIO, com debounce.
 *
 * Ligação: reed entre o GPIO e GND, pull-up externo para 3V3 (fechado = nível 0).
 *
 * Debounce por tempo mínimo aberto: um fechamento (borda de descida) só é
 * contado se a linha ficou aberta (nível 1) por pelo menos `min_open_us`.
 * Isso rejeita tanto o bounce do fechamento quanto o da abertura, inclusive
 * em baixa rotação, quando o reed fica fechado por centenas de ms e o bounce
 * da abertura escaparia de um simples lockout após a borda de descida.
 *
 * Usado pelo anemômetro e pelo pluviômetro basculante.
 */
#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>

typedef struct {
    gpio_num_t        gpio;
    uint32_t          min_open_us;  /* Tempo mínimo aberto para aceitar um fechamento */
    volatile int64_t  t_open_us;    /* Instante da última abertura (borda de subida)  */
    volatile uint32_t count;        /* Fechamentos válidos desde o init (só a ISR escreve) */
} reed_counter_t;

/**
 * @brief Configura o GPIO (entrada, interrupção em ambas as bordas) e registra a ISR.
 *        Instala o serviço de ISR de GPIO se ainda não estiver instalado.
 * @param rc           Instância (deve permanecer válida enquanto a ISR estiver ativa)
 * @param gpio         GPIO ligado ao reed
 * @param min_open_us  Debounce: tempo mínimo aberto antes de um fechamento válido
 */
esp_err_t reed_counter_init(reed_counter_t *rc, gpio_num_t gpio, uint32_t min_open_us);

/** @brief Total de pulsos válidos desde o init (leitura 32 bits alinhada é atômica). */
static inline uint32_t reed_counter_get(const reed_counter_t *rc)
{
    return rc->count;
}
