#include "lora_sx1262.h"
#include "pin_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sx1262";

/* ---------- Comandos SX1262 (datasheet Semtech DS.SX1261-2.W.APP Rev 2.1) ---------- */
#define SX_CMD_SET_SLEEP            0x84
#define SX_CMD_SET_STANDBY          0x80
#define SX_CMD_SET_TX               0x83
#define SX_CMD_SET_RX               0x82
#define SX_CMD_SET_PKT_TYPE         0x8A
#define SX_CMD_SET_RF_FREQ          0x86
#define SX_CMD_SET_PA_CONFIG        0x95
#define SX_CMD_SET_TX_PARAMS        0x8E
#define SX_CMD_SET_BUF_BASE_ADDR    0x8F
#define SX_CMD_SET_MOD_PARAMS       0x8B
#define SX_CMD_SET_PKT_PARAMS       0x8C
#define SX_CMD_SET_DIO_IRQ_PARAMS   0x08
#define SX_CMD_CLR_IRQ_STATUS       0x02
#define SX_CMD_GET_IRQ_STATUS       0x12
#define SX_CMD_WRITE_BUFFER         0x0E
#define SX_CMD_WRITE_REGISTER       0x0D
#define SX_CMD_READ_REGISTER        0x1D
#define SX_CMD_SET_DIO3_AS_TCXO     0x97
#define SX_CMD_CALIBRATE            0x89
#define SX_CMD_SET_DIO2_AS_RF_SW    0x9D
#define SX_CMD_SET_REGULATOR_MODE   0x96
#define SX_CMD_GET_STATUS           0xC0

/* Modos de standby */
#define SX_STDBY_RC     0x00
#define SX_STDBY_XOSC   0x01

/* Tipo de pacote */
#define SX_PKT_TYPE_LORA  0x01

/* IRQ masks */
#define SX_IRQ_TX_DONE      (1 << 0)
#define SX_IRQ_RX_DONE      (1 << 1)
#define SX_IRQ_TIMEOUT      (1 << 9)
#define SX_IRQ_ALL          0x03FF

/*
 * Workaround "Modulation Quality with 500 kHz LoRa Bandwidth" (datasheet 15.1):
 * bit 2 do registrador 0x0889 em 0 para BW = 500 kHz e em 1 nas demais.
 */
#define SX_REG_TX_MODULATION  0x0889
#define SX_TX_MOD_BIT         (1U << 2)

#define SPI_TIMEOUT_MS  100
#define TX_TIMEOUT_MS   5000

/* ---------- SPI helpers ---------- */

static spi_device_handle_t spi_dev;

static esp_err_t wait_busy(void)
{
    int tries = 1000;
    while (gpio_get_level(LORA_PIN_BUSY) && --tries > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return (tries > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static esp_err_t spi_cmd(const uint8_t *cmd, size_t cmd_len,
                         uint8_t *rx, size_t rx_len)
{
    esp_err_t ret;

    ret = wait_busy();
    if (ret != ESP_OK) { return ret; }

    /* TX: comando + NOP bytes para receber resposta */
    size_t total = cmd_len + rx_len;
    uint8_t tx_buf[32] = {0};
    uint8_t rx_buf[32] = {0};
    memcpy(tx_buf, cmd, cmd_len);

    spi_transaction_t txn = {
        .length    = total * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    ret = spi_device_transmit(spi_dev, &txn);
    if (ret != ESP_OK) { return ret; }

    if (rx != NULL && rx_len > 0) {
        memcpy(rx, &rx_buf[cmd_len], rx_len);
    }
    return ESP_OK;
}

static esp_err_t spi_write_cmd(const uint8_t *cmd, size_t len)
{
    return spi_cmd(cmd, len, NULL, 0);
}

/* ---------- Comandos de alto nível ---------- */

static esp_err_t sx_set_standby(uint8_t mode)
{
    uint8_t cmd[] = { SX_CMD_SET_STANDBY, mode };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_packet_type(uint8_t type)
{
    uint8_t cmd[] = { SX_CMD_SET_PKT_TYPE, type };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_rf_frequency(uint32_t freq_hz)
{
    /* freq_reg = freq_hz * 2^25 / 32MHz */
    uint32_t freq_reg = (uint32_t)((uint64_t)freq_hz * (1 << 25) / 32000000UL);
    uint8_t cmd[] = {
        SX_CMD_SET_RF_FREQ,
        (uint8_t)(freq_reg >> 24),
        (uint8_t)(freq_reg >> 16),
        (uint8_t)(freq_reg >> 8),
        (uint8_t)(freq_reg),
    };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_pa_config(void)
{
    /* PA: SX1262 high power: paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00, paLut=0x01 */
    uint8_t cmd[] = { SX_CMD_SET_PA_CONFIG, 0x04, 0x07, 0x00, 0x01 };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_tx_params(uint8_t power_dbm, uint8_t ramp_time)
{
    uint8_t cmd[] = { SX_CMD_SET_TX_PARAMS, power_dbm, ramp_time };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_buffer_base_address(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t cmd[] = { SX_CMD_SET_BUF_BASE_ADDR, tx_base, rx_base };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_modulation_params(uint8_t sf, uint8_t bw, uint8_t cr,
                                          uint8_t ldro)
{
    uint8_t cmd[] = { SX_CMD_SET_MOD_PARAMS, sf, bw, cr, ldro };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_packet_params(uint16_t preamble, uint8_t header_type,
                                      uint8_t payload_len, uint8_t crc_on,
                                      uint8_t invert_iq)
{
    uint8_t cmd[] = {
        SX_CMD_SET_PKT_PARAMS,
        (uint8_t)(preamble >> 8), (uint8_t)preamble,
        header_type, payload_len, crc_on, invert_iq,
    };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask,
                                       uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t cmd[] = {
        SX_CMD_SET_DIO_IRQ_PARAMS,
        (uint8_t)(irq_mask >> 8),  (uint8_t)irq_mask,
        (uint8_t)(dio1_mask >> 8), (uint8_t)dio1_mask,
        (uint8_t)(dio2_mask >> 8), (uint8_t)dio2_mask,
        (uint8_t)(dio3_mask >> 8), (uint8_t)dio3_mask,
    };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_clear_irq_status(uint16_t mask)
{
    uint8_t cmd[] = { SX_CMD_CLR_IRQ_STATUS, (uint8_t)(mask >> 8), (uint8_t)mask };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_get_irq_status(uint16_t *status)
{
    uint8_t cmd[] = { SX_CMD_GET_IRQ_STATUS };
    uint8_t rx[3] = {0}; /* 1 byte status + 2 bytes IRQ */
    esp_err_t ret = spi_cmd(cmd, sizeof(cmd), rx, 3);
    if (ret == ESP_OK) {
        *status = ((uint16_t)rx[1] << 8) | rx[2];
    }
    return ret;
}

static esp_err_t sx_write_buffer(uint8_t offset, const uint8_t *data, uint8_t len)
{
    esp_err_t ret;

    ret = wait_busy();
    if (ret != ESP_OK) { return ret; }

    /* Monta: [CMD][OFFSET][DATA...] */
    uint8_t tx_buf[2 + 255];
    tx_buf[0] = SX_CMD_WRITE_BUFFER;
    tx_buf[1] = offset;
    memcpy(&tx_buf[2], data, len);

    spi_transaction_t txn = {
        .length    = (2 + len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = NULL,
    };
    return spi_device_transmit(spi_dev, &txn);
}

static esp_err_t sx_write_register(uint16_t addr, uint8_t val)
{
    uint8_t cmd[] = {
        SX_CMD_WRITE_REGISTER,
        (uint8_t)(addr >> 8), (uint8_t)addr,
        val,
    };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_read_register(uint16_t addr, uint8_t *val)
{
    /* [CMD][ADDR_H][ADDR_L][NOP→status] e então o dado */
    uint8_t cmd[] = { SX_CMD_READ_REGISTER, (uint8_t)(addr >> 8), (uint8_t)addr, 0x00 };
    return spi_cmd(cmd, sizeof(cmd), val, 1);
}

static esp_err_t sx_set_tx(uint32_t timeout_us)
{
    /* timeout em unidades de 15.625µs */
    uint32_t t = timeout_us * 64 / 1000;
    uint8_t cmd[] = {
        SX_CMD_SET_TX,
        (uint8_t)(t >> 16), (uint8_t)(t >> 8), (uint8_t)t,
    };
    return spi_write_cmd(cmd, sizeof(cmd));
}

/* ---------- Configurações específicas do Heltec V3 ---------- */

static esp_err_t sx_set_dio3_as_tcxo(void)
{
    /* Heltec V3: TCXO a 1.7V via DIO3, timeout 5ms */
    uint8_t cmd[] = { SX_CMD_SET_DIO3_AS_TCXO, 0x01, 0x00, 0x01, 0x40 };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_calibrate_all(void)
{
    uint8_t cmd[] = { SX_CMD_CALIBRATE, 0x7F };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_dio2_as_rf_switch(void)
{
    uint8_t cmd[] = { SX_CMD_SET_DIO2_AS_RF_SW, 0x01 };
    return spi_write_cmd(cmd, sizeof(cmd));
}

static esp_err_t sx_set_regulator_mode_dcdc(void)
{
    uint8_t cmd[] = { SX_CMD_SET_REGULATOR_MODE, 0x01 };
    return spi_write_cmd(cmd, sizeof(cmd));
}

/* ---------- Init ---------- */

esp_err_t lora_sx1262_init(lora_sx1262_handle_t *handle)
{
    esp_err_t ret;

    /* GPIO: RST (output), BUSY (input), DIO1 (input) */
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << LORA_PIN_RST,
        .mode         = GPIO_MODE_OUTPUT,
    };
    ret = gpio_config(&out_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar RST: %s", esp_err_to_name(ret));
        goto err_gpio;
    }

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << LORA_PIN_BUSY) | (1ULL << LORA_PIN_DIO1),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ret = gpio_config(&in_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao configurar BUSY/DIO1: %s", esp_err_to_name(ret));
        goto err_gpio;
    }

    /* SPI bus init */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LORA_PIN_MOSI,
        .miso_io_num = LORA_PIN_MISO,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 300,
    };
    ret = spi_bus_initialize(LORA_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no init SPI: %s", esp_err_to_name(ret));
        goto err_spi;
    }

    /* SPI device (SX1262: max 16 MHz, mode 0) */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 8000000,      /* 8 MHz */
        .mode           = 0,
        .spics_io_num   = LORA_PIN_CS,
        .queue_size     = 1,
    };
    ret = spi_bus_add_device(LORA_SPI_HOST, &dev_cfg, &spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao adicionar SPI device: %s", esp_err_to_name(ret));
        goto err_dev;
    }
    handle->spi_dev = spi_dev;

    /* Hardware reset */
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ret = wait_busy();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SX1262 BUSY nao liberou apos reset");
        goto err_cfg;
    }

    /* Sequência de configuração (ordem importa!) */
    ret = sx_set_standby(SX_STDBY_RC);
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_dio3_as_tcxo();
    if (ret != ESP_OK) { goto err_cfg; }
    vTaskDelay(pdMS_TO_TICKS(5));

    ret = sx_calibrate_all();
    if (ret != ESP_OK) { goto err_cfg; }
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = sx_set_standby(SX_STDBY_XOSC);
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_regulator_mode_dcdc();
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_dio2_as_rf_switch();
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_packet_type(SX_PKT_TYPE_LORA);
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_rf_frequency(LORA_FREQUENCY_HZ);
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_pa_config();
    if (ret != ESP_OK) { goto err_cfg; }

    /* TX power 22 dBm, ramp 200µs */
    ret = sx_set_tx_params(LORA_TX_POWER, 0x04);
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_set_buffer_base_address(0x00, 0x80);
    if (ret != ESP_OK) { goto err_cfg; }

    /* LDRO auto: desligado para SF7/125kHz (Tsym < 16ms) */
    ret = sx_set_modulation_params(LORA_SF, LORA_BW, LORA_CR, 0x00);
    if (ret != ESP_OK) { goto err_cfg; }

    /* Workaround de BW (ver SX_REG_TX_MODULATION): ler, ajustar o bit 2, escrever */
    uint8_t txmod;
    ret = sx_read_register(SX_REG_TX_MODULATION, &txmod);
    if (ret != ESP_OK) { goto err_cfg; }
    txmod = (LORA_BW == 0x06) ? (uint8_t)(txmod & ~SX_TX_MOD_BIT) : (uint8_t)(txmod | SX_TX_MOD_BIT);
    ret = sx_write_register(SX_REG_TX_MODULATION, txmod);
    if (ret != ESP_OK) { goto err_cfg; }

    /* Pacote: preamble 8, header explícito, CRC on, IQ normal */
    ret = sx_set_packet_params(LORA_PREAMBLE_LEN, 0x00, 0xFF, 0x01, 0x00);
    if (ret != ESP_OK) { goto err_cfg; }

    /* IRQ: TX_DONE e TIMEOUT no DIO1 */
    ret = sx_set_dio_irq_params(
        SX_IRQ_TX_DONE | SX_IRQ_TIMEOUT,
        SX_IRQ_TX_DONE | SX_IRQ_TIMEOUT,
        0x0000, 0x0000
    );
    if (ret != ESP_OK) { goto err_cfg; }

    ret = sx_clear_irq_status(SX_IRQ_ALL);
    if (ret != ESP_OK) { goto err_cfg; }

    ESP_LOGI(TAG, "SX1262 inicializado (freq=%lu Hz, SF%d, BW=125kHz, TX=%ddBm)",
             (unsigned long)LORA_FREQUENCY_HZ, LORA_SF, LORA_TX_POWER);
    return ESP_OK;

err_cfg:
    spi_bus_remove_device(spi_dev);
err_dev:
    spi_bus_free(LORA_SPI_HOST);
err_spi:
err_gpio:
    return ret;
}

/* ---------- Transmissão ---------- */

esp_err_t lora_sx1262_transmit(lora_sx1262_handle_t *handle,
                               const uint8_t *data, uint8_t len)
{
    esp_err_t ret;

    /* Atualiza tamanho do payload */
    ret = sx_set_packet_params(LORA_PREAMBLE_LEN, 0x00, len, 0x01, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao definir packet params: %s", esp_err_to_name(ret));
        goto err_tx;
    }

    /* Escreve dados no buffer TX do SX1262 */
    ret = sx_write_buffer(0x00, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao escrever buffer: %s", esp_err_to_name(ret));
        goto err_tx;
    }

    /* Limpa IRQs e inicia TX (timeout 5s) */
    ret = sx_clear_irq_status(SX_IRQ_ALL);
    if (ret != ESP_OK) { goto err_tx; }

    ret = sx_set_tx(TX_TIMEOUT_MS * 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar TX: %s", esp_err_to_name(ret));
        goto err_tx;
    }

    /* Aguarda DIO1 (TX_DONE ou TIMEOUT) */
    int elapsed = 0;
    while (!gpio_get_level(LORA_PIN_DIO1) && elapsed < TX_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(5));
        elapsed += 5;
    }

    /* Verifica resultado */
    uint16_t irq = 0;
    ret = sx_get_irq_status(&irq);
    if (ret != ESP_OK) { goto err_tx; }

    ret = sx_clear_irq_status(SX_IRQ_ALL);
    if (ret != ESP_OK) { goto err_tx; }

    if (irq & SX_IRQ_TX_DONE) {
        ESP_LOGD(TAG, "TX OK (%d bytes)", len);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "TX falhou (IRQ=0x%04X, elapsed=%dms)", irq, elapsed);
    ret = ESP_ERR_TIMEOUT;

err_tx:
    /* Volta para standby em caso de erro */
    sx_set_standby(SX_STDBY_RC);
    return ret;
}
