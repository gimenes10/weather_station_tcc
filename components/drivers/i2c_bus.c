#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "driver/gpio.h"
#include <stdio.h>

static const char *TAG = "i2c_bus";

/* A 10 kHz cada byte leva ~0,9 ms; a maior leitura (BME680, 17 bytes) ~20 ms */
#define I2C_TIMEOUT_MS  100

/* Scan: um NACK a 10 kHz leva ~2 ms. Bem mais que isso = o controlador não
   conseguiu gerar o clock (linha presa em 0 ou subindo devagar demais). */
#define SCAN_TIMEOUT_MS 20
#define SCAN_STUCK_US   10000
#define SCAN_SLOW_MAX   3       /* Tentativas lentas antes de desistir da varredura */

/*
 * Diagnóstico: nível que o próprio ESP32 lê em SDA/SCL, com pull-up interno,
 * antes de o controlador I2C assumir os pinos. Diferente do multímetro, não
 * mede média de tráfego: 0 aqui = linha realmente presa em nível baixo.
 */
static void pin_check(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << SENSOR_I2C_SDA) | (1ULL << SENSOR_I2C_SCL),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    gpio_config(&io);
    esp_rom_delay_us(2000);
    ESP_LOGI(TAG, "Pinos antes do I2C: SDA(GPIO%d)=%d SCL(GPIO%d)=%d (esperado 1 e 1)",
             SENSOR_I2C_SDA, gpio_get_level(SENSOR_I2C_SDA),
             SENSOR_I2C_SCL, gpio_get_level(SENSOR_I2C_SCL));
}

/*
 * Teste de cada linha como GPIO open-drain, antes do driver I2C:
 *  - força a 0 e lê de volta (o pino consegue gerar o clock/dado?)
 *  - lê a outra linha nesse instante (SDA e SCL em curto?)
 *  - solta e mede a subida até voltar a 1 (borda lenta ou algo segurando?)
 */
#define LINE_RISE_MAX_NS   2000000   /* 2 ms: não voltou = linha presa */
#define LINE_RISE_SLOW_NS  10000     /* A 5 kHz o meio período é 100 µs; acima de 10 µs já é suspeito */

static void line_test(void)
{
    static const struct { gpio_num_t pin, other; const char *name; } L[] = {
        { SENSOR_I2C_SDA, SENSOR_I2C_SCL, "SDA" },
        { SENSOR_I2C_SCL, SENSOR_I2C_SDA, "SCL" },
    };
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << SENSOR_I2C_SDA) | (1ULL << SENSOR_I2C_SCL),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    const uint32_t ticks_us = esp_rom_get_cpu_ticks_per_us();

    gpio_config(&io);
    gpio_set_level(SENSOR_I2C_SDA, 1);
    gpio_set_level(SENSOR_I2C_SCL, 1);
    esp_rom_delay_us(200);

    for (size_t i = 0; i < sizeof(L) / sizeof(L[0]); i++) {
        const int idle = gpio_get_level(L[i].pin);

        gpio_set_level(L[i].pin, 0);
        esp_rom_delay_us(20);
        const int low   = gpio_get_level(L[i].pin);
        const int other = gpio_get_level(L[i].other);

        const uint32_t t0 = esp_cpu_get_cycle_count();
        uint32_t       dt = 0;
        gpio_set_level(L[i].pin, 1);
        while (!gpio_get_level(L[i].pin)) {
            dt = esp_cpu_get_cycle_count() - t0;
            if (dt / ticks_us * 1000U > LINE_RISE_MAX_NS) {
                break;
            }
        }
        dt = esp_cpu_get_cycle_count() - t0;
        const uint32_t rise_ns = dt * 1000U / ticks_us;
        const bool     stuck   = !gpio_get_level(L[i].pin);

        const char *diag = !idle             ? "presa em 0 em repouso"
                         : low               ? "o pino nao consegue baixar a linha (curto com 3V3?)"
                         : !other            ? "a outra linha caiu junto: SDA e SCL em curto"
                         : stuck             ? "nao volta a 1: sensor ou curto segurando a linha"
                         : rise_ns > LINE_RISE_SLOW_NS ? "borda lenta: capacitancia alta ou pull-up fraco"
                                                       : "OK";
        ESP_LOGI(TAG, "Teste %s: repouso=%d, forcada a 0 le %d, outra linha=%d, sobe em %s%lu ns -> %s",
                 L[i].name, idle, low, other, stuck ? ">" : "", (unsigned long)rise_ns, diag);
        esp_rom_delay_us(50);
    }

    /* Devolve os pinos ao estado de boot antes de o driver I2C assumi-los */
    gpio_reset_pin(SENSOR_I2C_SDA);
    gpio_reset_pin(SENSOR_I2C_SCL);
}

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus)
{
    pin_check();
    line_test();

    const i2c_master_bus_config_t cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = SENSOR_I2C_PORT,
        .scl_io_num        = SENSOR_I2C_SCL,
        .sda_io_num        = SENSOR_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,    /* ~45 kΩ: garantem nível alto; o pull-up principal é o externo */
    };
    esp_err_t ret = i2c_new_master_bus(&cfg, bus);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar o barramento: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_bus_reset(i2c_master_bus_handle_t bus)
{
    return i2c_master_bus_reset(bus);
}

#define RECOVER_HALF_US  50      /* Meio período de 10 kHz */
#define RECOVER_PULSES   16      /* 9 bastam para um byte; 16 cobrem dois */

esp_err_t i2c_bus_recover(i2c_master_bus_handle_t *bus)
{
    /* Apaga o driver para os pinos voltarem a ser GPIO. Falha se ainda houver
       dispositivo registrado: os drivers dos sensores soltam os seus antes. */
    esp_err_t ret = ESP_OK;

    if (*bus != NULL) {   /* NULL: a recriação anterior falhou */
        ret = i2c_del_master_bus(*bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Recuperacao: nao consegui apagar o barramento (%s)", esp_err_to_name(ret));
            goto err;
        }
        *bus = NULL;
    }

    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << SENSOR_I2C_SDA) | (1ULL << SENSOR_I2C_SCL),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(SENSOR_I2C_SDA, 1);
    gpio_set_level(SENSOR_I2C_SCL, 1);
    esp_rom_delay_us(RECOVER_HALF_US);

    const int sda0 = gpio_get_level(SENSOR_I2C_SDA);
    const int scl0 = gpio_get_level(SENSOR_I2C_SCL);
    int       n    = 0;

    /* Pulsos no SCL até o escravo terminar o byte que acha que está enviando */
    while (!gpio_get_level(SENSOR_I2C_SDA) && n < RECOVER_PULSES) {
        gpio_set_level(SENSOR_I2C_SCL, 0);
        esp_rom_delay_us(RECOVER_HALF_US);
        gpio_set_level(SENSOR_I2C_SCL, 1);
        esp_rom_delay_us(RECOVER_HALF_US);
        n++;
    }

    /* STOP: SDA sobe com SCL alto, todos os escravos voltam ao repouso */
    gpio_set_level(SENSOR_I2C_SCL, 0);
    esp_rom_delay_us(RECOVER_HALF_US);
    gpio_set_level(SENSOR_I2C_SDA, 0);
    esp_rom_delay_us(RECOVER_HALF_US);
    gpio_set_level(SENSOR_I2C_SCL, 1);
    esp_rom_delay_us(RECOVER_HALF_US);
    gpio_set_level(SENSOR_I2C_SDA, 1);
    esp_rom_delay_us(RECOVER_HALF_US);

    const int sda1 = gpio_get_level(SENSOR_I2C_SDA);
    const int scl1 = gpio_get_level(SENSOR_I2C_SCL);

    ESP_LOGW(TAG, "Recuperacao: antes SDA=%d SCL=%d, %d pulsos em SCL, depois SDA=%d SCL=%d%s",
             sda0, scl0, n, sda1, scl1,
             (sda1 && scl1) ? "" : " -> continuam presas: sensor travado sem soltar ou fuga para o GND (umidade)");

    /* Devolve os pinos ao estado de boot; sem isso o driver I2C acusa conflito ao reassumi-los */
    gpio_reset_pin(SENSOR_I2C_SDA);
    gpio_reset_pin(SENSOR_I2C_SCL);

    ret = i2c_bus_init(bus);
err:
    return ret;
}

/* Nível atual das linhas (o driver deixa a entrada habilitada) e o diagnóstico */
static void log_lines(const char *when)
{
    const int sda = gpio_get_level(SENSOR_I2C_SDA);
    const int scl = gpio_get_level(SENSOR_I2C_SCL);

    ESP_LOGW(TAG, "%s: SDA=%d SCL=%d -> %s", when, sda, scl,
             (sda && scl) ? "linhas em 1: pull-up fraco ou borda lenta (falta pull-up externo?)"
             : !scl       ? "SCL preso em 0: dispositivo segurando o clock ou SCL em curto"
                          : "SDA preso em 0: dispositivo travado no meio de uma transacao");
}

int i2c_bus_scan(i2c_master_bus_handle_t bus)
{
    /*
     * Lê 1 byte de cada endereço na frequência do barramento (10 kHz).
     * O i2c_master_probe() do IDF força 100 kHz, o que daria falso negativo
     * justamente com cabo longo ou pull-up fraco. Os erros de NACK do driver
     * são silenciados durante a varredura.
     *
     * No IDF 6.1 o driver devolve ESP_ERR_INVALID_RESPONSE tanto para NACK
     * quanto para timeout; a diferença aparece no tempo de cada tentativa.
     */
    const esp_log_level_t lvl = esp_log_level_get("i2c.master");
    char      list[3 * 112 + 1] = "";
    int       found     = 0;
    int       nack      = 0;
    int       slow      = 0;
    uint8_t   slow_addr = 0;
    int64_t   slow_us   = 0;

    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_master_dev_handle_t dev;
        uint8_t byte;

        if (i2c_bus_add(bus, addr, &dev) != ESP_OK) {
            continue;
        }
        const int64_t   t0  = esp_timer_get_time();
        const esp_err_t ret = i2c_master_receive(dev, &byte, 1, SCAN_TIMEOUT_MS);
        const int64_t   dt  = esp_timer_get_time() - t0;
        i2c_master_bus_rm_device(dev);

        if (ret == ESP_OK) {
            snprintf(&list[3 * found], sizeof(list) - 3 * found, " %02X", addr);
            found++;
        } else if (dt > SCAN_STUCK_US) {
            if (slow == 0) {
                slow_addr = addr;
                slow_us   = dt;
            }
            if (++slow >= SCAN_SLOW_MAX) {
                break;   /* Barramento sem clock: não adianta testar o resto */
            }
        } else {
            nack++;
        }
    }
    esp_log_level_set("i2c.master", lvl);

    ESP_LOGI(TAG, "Scan: %d ACK%s | %d NACK | %d lentas (BME680=76/77, BH1750=23/5C)",
             found, found ? list : "", nack, slow);
    if (slow) {
        ESP_LOGE(TAG, "Scan: 0x%02X levou %lld ms (NACK normal ~1 ms)", slow_addr, (long long)(slow_us / 1000));
        log_lines("Linhas depois da tentativa lenta");
    }
    return found;
}

esp_err_t i2c_bus_add(i2c_master_bus_handle_t bus, uint8_t addr, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = SENSOR_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus, &cfg, dev);
}

esp_err_t i2c_reg_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

esp_err_t i2c_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}
