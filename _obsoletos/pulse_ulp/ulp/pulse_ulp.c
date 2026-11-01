/**
 * @file pulse_ulp.c
 * @brief Programa do ULP RISC-V: conta os reeds do anemômetro e do pluviômetro
 *        e calcula a rajada enquanto o CPU principal dorme.
 *
 * O timer do ULP reinicia este programa a cada período (configurado pelo CPU).
 * Cada execução amostra os dois pinos uma vez e termina. As variáveis globais
 * ficam na RTC slow memory e persistem entre execuções; o CPU as acessa com o
 * prefixo ulp_ (ex.: ulp_wind_count).
 *
 * Debounce por tempo mínimo aberto: um fechamento só conta se a linha ficou em
 * nível alto por `min_open` amostras seguidas. Isso rejeita o bounce do
 * fechamento e o da abertura, inclusive em baixa rotação.
 *
 * Rajada (WMO): maior soma de pulsos em GUST_SLOTS janelas consecutivas de 1 s.
 */
#include <stdint.h>
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"

#define GUST_SLOTS  3U

/* Configuração escrita pelo CPU antes de ulp_riscv_run() */
uint32_t cfg_wind_gpio;
uint32_t cfg_rain_gpio;
uint32_t cfg_wind_min_open;     /* amostras */
uint32_t cfg_rain_min_open;
uint32_t cfg_slot_ticks;        /* execuções por janela de ~1 s */

/* Saídas (monotônicas; o CPU trata o estouro de 32 bits por diferença) */
uint32_t wind_count;
uint32_t rain_count;
uint32_t tick_count;
uint32_t gust_max;              /* maior soma de GUST_SLOTS janelas desde o último reset */
uint32_t gust_reset;            /* CPU escreve 1; o ULP zera gust_max e devolve 0 */
uint32_t gust_slots = GUST_SLOTS;

static uint32_t wind_open;
static uint32_t rain_open;
static uint32_t slot_ticks;
static uint32_t slot_base;
static uint32_t slots[GUST_SLOTS];
static uint32_t slot_idx;

static void sample(uint32_t gpio, uint32_t min_open, uint32_t *open_run, uint32_t *count)
{
    if (ulp_riscv_gpio_get_level((gpio_num_t)gpio)) {
        if (*open_run < min_open) {
            (*open_run)++;
        }
        return;
    }
    if (*open_run >= min_open) {
        (*count)++;
    }
    *open_run = 0;
}

int main(void)
{
    if (gust_reset) {
        gust_max   = 0;
        gust_reset = 0;
    }

    sample(cfg_wind_gpio, cfg_wind_min_open, &wind_open, &wind_count);
    sample(cfg_rain_gpio, cfg_rain_min_open, &rain_open, &rain_count);
    tick_count++;

    if (++slot_ticks >= cfg_slot_ticks) {
        uint32_t sum = 0;

        slot_ticks      = 0;
        slots[slot_idx] = wind_count - slot_base;
        slot_base       = wind_count;
        slot_idx        = (slot_idx + 1U) % GUST_SLOTS;

        /* gust_slots (= GUST_SLOTS) é lido aqui para o linker manter o símbolo:
           o CPU usa o mesmo valor ao converter a rajada */
        for (uint32_t i = 0; i < gust_slots; i++) {
            sum += slots[i];
        }
        if (sum > gust_max) {
            gust_max = sum;
        }
    }
    return 0;   /* start.S chama ulp_riscv_halt(); o timer reinicia o programa */
}
