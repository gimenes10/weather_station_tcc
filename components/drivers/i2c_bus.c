#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
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

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus)
{
    pin_check();

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
