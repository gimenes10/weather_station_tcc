#include "bme680.h"
#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "bme680";

/* ---------- Registradores (datasheet BST-BME680-DS001) ---------- */
#define REG_COEFF3          0x00
#define REG_FIELD0          0x1D    /* meas_status_0 .. gas_r_lsb (BME688) */
#define REG_RES_HEAT_0      0x5A
#define REG_GAS_WAIT_0      0x64
#define REG_CTRL_GAS_1      0x71
#define REG_CTRL_HUM        0x72
#define REG_CTRL_MEAS       0x74
#define REG_CONFIG          0x75
#define REG_COEFF1          0x8A
#define REG_CHIP_ID         0xD0
#define REG_RESET           0xE0
#define REG_COEFF2          0xE1
#define REG_VARIANT_ID      0xF0

#define CHIP_ID             0x61
#define VARIANT_GAS_HIGH    0x01    /* BME688 */
#define SOFT_RESET_CMD      0xB6

#define LEN_COEFF1          23
#define LEN_COEFF2          14
#define LEN_COEFF3          5
#define LEN_FIELD           17

#define NEW_DATA_MSK        0x80
#define GAS_VALID_MSK       0x20
#define HEAT_STAB_MSK       0x10
#define GAS_RANGE_MSK       0x0F

/* ---------- Configuração da medição ---------- */
#define OS_T                2       /* ×2 */
#define OS_P                3       /* ×4 */
#define OS_H                1       /* ×1 */
#define HEATER_TEMP_C       320
#define HEATER_MS           150
#define MODE_FORCED         0x01
#define POLL_MS             5
#define POLL_TIMEOUT_MS     100

#define CAL_MAGIC           0x0680CA1BU

typedef struct {
    uint16_t par_t1;  int16_t par_t2;  int8_t par_t3;
    uint16_t par_p1;  int16_t par_p2;  int8_t par_p3;  int16_t par_p4; int16_t par_p5;
    int8_t   par_p6;  int8_t  par_p7;  int16_t par_p8; int16_t par_p9; uint8_t par_p10;
    uint16_t par_h1;  uint16_t par_h2; int8_t par_h3;  int8_t par_h4;  int8_t par_h5;
    uint8_t  par_h6;  int8_t  par_h7;
    int8_t   par_gh1; int16_t par_gh2; int8_t par_gh3;
    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
    uint8_t  variant;
} calib_t;

/* Calibração (lida uma vez) e última temperatura, usada no cálculo do aquecedor */
static calib_t  s_cal;
static uint32_t s_cal_magic;
static float    s_amb_c = 25.0f;

static i2c_master_dev_handle_t s_dev;

/* ---------- Calibração ---------- */

static esp_err_t load_calibration(void)
{
    uint8_t   c[LEN_COEFF1 + LEN_COEFF2 + LEN_COEFF3];
    esp_err_t ret;

    ret = i2c_reg_write8(s_dev, REG_RESET, SOFT_RESET_CMD);
    if (ret != ESP_OK) { goto err; }
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = i2c_reg_read(s_dev, REG_COEFF1, &c[0], LEN_COEFF1);
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_read(s_dev, REG_COEFF2, &c[LEN_COEFF1], LEN_COEFF2);
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_read(s_dev, REG_COEFF3, &c[LEN_COEFF1 + LEN_COEFF2], LEN_COEFF3);
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_read(s_dev, REG_VARIANT_ID, &s_cal.variant, 1);
    if (ret != ESP_OK) { goto err; }

    /* Índices da BME68x API (bme68x_defs.h) */
    s_cal.par_t1  = (uint16_t)((c[32] << 8) | c[31]);
    s_cal.par_t2  = (int16_t)((c[1] << 8) | c[0]);
    s_cal.par_t3  = (int8_t)c[2];
    s_cal.par_p1  = (uint16_t)((c[5] << 8) | c[4]);
    s_cal.par_p2  = (int16_t)((c[7] << 8) | c[6]);
    s_cal.par_p3  = (int8_t)c[8];
    s_cal.par_p4  = (int16_t)((c[11] << 8) | c[10]);
    s_cal.par_p5  = (int16_t)((c[13] << 8) | c[12]);
    s_cal.par_p6  = (int8_t)c[15];
    s_cal.par_p7  = (int8_t)c[14];
    s_cal.par_p8  = (int16_t)((c[19] << 8) | c[18]);
    s_cal.par_p9  = (int16_t)((c[21] << 8) | c[20]);
    s_cal.par_p10 = c[22];
    s_cal.par_h1  = (uint16_t)((c[25] << 4) | (c[24] & 0x0F));
    s_cal.par_h2  = (uint16_t)((c[23] << 4) | (c[24] >> 4));
    s_cal.par_h3  = (int8_t)c[26];
    s_cal.par_h4  = (int8_t)c[27];
    s_cal.par_h5  = (int8_t)c[28];
    s_cal.par_h6  = c[29];
    s_cal.par_h7  = (int8_t)c[30];
    s_cal.par_gh1 = (int8_t)c[35];
    s_cal.par_gh2 = (int16_t)((c[34] << 8) | c[33]);
    s_cal.par_gh3 = (int8_t)c[36];
    s_cal.res_heat_val   = (int8_t)c[37];
    s_cal.res_heat_range = (uint8_t)((c[39] & 0x30) >> 4);
    s_cal.range_sw_err   = (int8_t)((int8_t)(c[41] & 0xF0) / 16);

    s_cal_magic = CAL_MAGIC;
    ESP_LOGI(TAG, "Calibracao lida (%s)", s_cal.variant == VARIANT_GAS_HIGH ? "BME688" : "BME680");
    return ESP_OK;

err:
    s_cal_magic = 0;
    return ret;
}

/* Adiciona o sensor no endereço e confere o chip ID */
static esp_err_t try_addr(i2c_master_bus_handle_t bus, uint8_t addr)
{
    uint8_t   id  = 0;
    esp_err_t ret = i2c_bus_add(bus, addr, &s_dev);

    if (ret != ESP_OK) {
        goto err_add;
    }
    /* Soft reset (0xB6 em 0xE0): tira o chip de qualquer estado travado, como
       desligar e ligar. A calibração fica na NVM e a configuração é regravada
       a cada disparo. No endereço errado a escrita dá NACK e o teste segue. */
    ret = i2c_reg_write8(s_dev, 0xE0, 0xB6);
    if (ret != ESP_OK) {
        goto err_dev;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    ret = i2c_reg_read(s_dev, REG_CHIP_ID, &id, 1);
    if (ret != ESP_OK) {
        goto err_dev;
    }
    if (id != CHIP_ID) {
        /* Comum em módulos vendidos como BME680: BME280 (0x60) ou BMP280 (0x58) */
        ESP_LOGE(TAG, "0x%02X respondeu com chip ID 0x%02X (esperado 0x%02X)%s", addr, id, CHIP_ID,
                 id == 0x60 ? ": e um BME280" : id == 0x58 ? ": e um BMP280" : "");
        ret = ESP_ERR_INVALID_RESPONSE;
        goto err_dev;
    }
    ESP_LOGI(TAG, "BME680 em 0x%02X", addr);
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(s_dev);
err_add:
    s_dev = NULL;
    return ret;
}

esp_err_t bme680_init(i2c_master_bus_handle_t bus)
{
    /* 0x76 com SDO no GND; 0x77 com SDO em nível alto (padrão de vários módulos) */
    static const uint8_t addrs[] = { BME680_I2C_ADDR, BME680_I2C_ADDR_ALT };
    esp_err_t ret = ESP_FAIL;

    for (size_t i = 0; i < sizeof(addrs); i++) {
        const int64_t t0 = esp_timer_get_time();

        ret = try_addr(bus, addrs[i]);
        if (ret == ESP_OK) {
            break;
        }
        /* ~1 ms = NACK (ninguém no endereço); ~100 ms = barramento sem clock */
        ESP_LOGW(TAG, "0x%02X sem resposta: %s em %lld ms", addrs[i], esp_err_to_name(ret),
                 (long long)((esp_timer_get_time() - t0) / 1000));
    }
    if (ret != ESP_OK) {
        goto err_add;
    }

    if (s_cal_magic != CAL_MAGIC) {
        ret = load_calibration();
        if (ret != ESP_OK) {
            goto err_dev;
        }
    }
    return ESP_OK;

err_dev:
    i2c_master_bus_rm_device(s_dev);
err_add:
    s_dev = NULL;
    return ret;
}

/* ---------- Aquecedor ---------- */

static uint8_t calc_res_heat(uint16_t temp)
{
    const float var1 = ((float)s_cal.par_gh1 / 16.0f) + 49.0f;
    const float var2 = (((float)s_cal.par_gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    const float var3 = (float)s_cal.par_gh3 / 1024.0f;
    const float var4 = var1 * (1.0f + (var2 * (float)temp));
    const float var5 = var4 + (var3 * s_amb_c);

    return (uint8_t)(3.4f * ((var5 * (4.0f / (4.0f + (float)s_cal.res_heat_range)) *
                              (1.0f / (1.0f + ((float)s_cal.res_heat_val * 0.002f)))) - 25.0f));
}

static uint8_t calc_gas_wait(uint16_t ms)
{
    uint8_t factor = 0;

    if (ms >= 0xFC0) {
        return 0xFF;
    }
    while (ms > 0x3F) {
        ms /= 4;
        factor++;
    }
    return (uint8_t)(ms + factor * 64);
}

/* Ciclos de conversão por oversampling: {pulado, ×1, ×2, ×4, ×8, ×16} */
static uint32_t meas_dur_us(void)
{
    static const uint8_t cycles[] = { 0, 1, 2, 4, 8, 16 };
    uint32_t us = (uint32_t)(cycles[OS_T] + cycles[OS_P] + cycles[OS_H]) * 1963U;

    us += 477U * 4U;   /* troca T/P/H */
    us += 477U * 5U;   /* medição de gás */
    us += 1000U;       /* despertar do forced */
    return us;
}

void bme680_deinit(void)
{
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
}

esp_err_t bme680_trigger(uint32_t *wait_ms)
{
    const uint8_t run_gas = (s_cal.variant == VARIANT_GAS_HIGH) ? 0x20 : 0x10;
    esp_err_t     ret;

    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = i2c_reg_write8(s_dev, REG_CTRL_HUM, OS_H);
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_write8(s_dev, REG_CONFIG, 0x00);   /* filtro IIR desligado */
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_write8(s_dev, REG_RES_HEAT_0, calc_res_heat(HEATER_TEMP_C));
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_write8(s_dev, REG_GAS_WAIT_0, calc_gas_wait(HEATER_MS));
    if (ret != ESP_OK) { goto err; }
    ret = i2c_reg_write8(s_dev, REG_CTRL_GAS_1, run_gas);   /* nb_conv = 0 */
    if (ret != ESP_OK) { goto err; }
    /* ctrl_hum só vale após escrever ctrl_meas, que também dispara a medição */
    ret = i2c_reg_write8(s_dev, REG_CTRL_MEAS, (OS_T << 5) | (OS_P << 2) | MODE_FORCED);
    if (ret != ESP_OK) { goto err; }

    *wait_ms = (meas_dur_us() + 999U) / 1000U + HEATER_MS;
    return ESP_OK;

err:
    ESP_LOGW(TAG, "Disparo falhou (escrita de configuracao): %s", esp_err_to_name(ret));
    return ret;
}

/* ---------- Compensação (BME68x API, versão float) ---------- */

static float comp_temperature(uint32_t adc, float *t_fine)
{
    const float var1 = (((float)adc / 16384.0f) - ((float)s_cal.par_t1 / 1024.0f)) * (float)s_cal.par_t2;
    const float d    = ((float)adc / 131072.0f) - ((float)s_cal.par_t1 / 8192.0f);
    const float var2 = d * d * ((float)s_cal.par_t3 * 16.0f);

    *t_fine = var1 + var2;
    return *t_fine / 5120.0f;
}

static float comp_pressure(uint32_t adc, float t_fine)
{
    float var1 = (t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)s_cal.par_p6 / 131072.0f);
    float var3;
    float p;

    var2 = var2 + (var1 * (float)s_cal.par_p5 * 2.0f);
    var2 = (var2 / 4.0f) + ((float)s_cal.par_p4 * 65536.0f);
    var1 = ((((float)s_cal.par_p3 * var1 * var1) / 16384.0f) + ((float)s_cal.par_p2 * var1)) / 524288.0f;
    var1 = (1.0f + (var1 / 32768.0f)) * (float)s_cal.par_p1;
    if ((int)var1 == 0) {
        return 0.0f;
    }
    p    = 1048576.0f - (float)adc;
    p    = ((p - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = ((float)s_cal.par_p9 * p * p) / 2147483648.0f;
    var2 = p * ((float)s_cal.par_p8 / 32768.0f);
    var3 = (p / 256.0f) * (p / 256.0f) * (p / 256.0f) * ((float)s_cal.par_p10 / 131072.0f);
    return p + (var1 + var2 + var3 + ((float)s_cal.par_p7 * 128.0f)) / 16.0f;   /* Pa */
}

static float comp_humidity(uint16_t adc, float t_fine)
{
    const float tc   = t_fine / 5120.0f;
    const float var1 = (float)adc - (((float)s_cal.par_h1 * 16.0f) + (((float)s_cal.par_h3 / 2.0f) * tc));
    const float var2 = var1 * (((float)s_cal.par_h2 / 262144.0f) *
                               (1.0f + (((float)s_cal.par_h4 / 16384.0f) * tc) +
                                (((float)s_cal.par_h5 / 1048576.0f) * tc * tc)));
    const float var3 = (float)s_cal.par_h6 / 16384.0f;
    const float var4 = (float)s_cal.par_h7 / 2097152.0f;
    float h = var2 + ((var3 + (var4 * tc)) * var2 * var2);

    return (h > 100.0f) ? 100.0f : (h < 0.0f) ? 0.0f : h;
}

static float comp_gas_low(uint16_t adc, uint8_t range)
{
    static const float k1[16] = { 0, 0, 0, 0, 0, -1.0f, 0, -0.8f, 0, 0, -0.2f, -0.5f, 0, -1.0f, 0, 0 };
    static const float k2[16] = { 0, 0, 0, 0, 0.1f, 0.7f, 0, -0.8f, -0.1f, 0, 0, 0, 0, 0, 0, 0 };
    const float var1 = 1340.0f + (5.0f * (float)s_cal.range_sw_err);
    const float var2 = var1 * (1.0f + k1[range] / 100.0f);
    const float var3 = 1.0f + (k2[range] / 100.0f);

    return 1.0f / (var3 * 0.000000125f * (float)(1U << range) * ((((float)adc - 512.0f) / var2) + 1.0f));
}

static float comp_gas_high(uint16_t adc, uint8_t range)
{
    const uint32_t var1 = 262144U >> range;
    const int32_t  var2 = 4096 + ((int32_t)adc - 512) * 3;

    return 1000000.0f * (float)var1 / (float)var2;
}

esp_err_t bme680_read(bme680_data_t *out)
{
    uint8_t   b[LEN_FIELD];
    uint32_t  waited = 0;
    float     t_fine;
    esp_err_t ret;

    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    for (;;) {
        ret = i2c_reg_read(s_dev, REG_FIELD0, b, sizeof(b));
        if (ret != ESP_OK) { goto err; }
        if (b[0] & NEW_DATA_MSK) { break; }
        if (waited >= POLL_TIMEOUT_MS) {
            ret = ESP_ERR_TIMEOUT;
            goto err;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        waited += POLL_MS;
    }

    const uint32_t adc_p = ((uint32_t)b[2] << 12) | ((uint32_t)b[3] << 4) | (b[4] >> 4);
    const uint32_t adc_t = ((uint32_t)b[5] << 12) | ((uint32_t)b[6] << 4) | (b[7] >> 4);
    const uint16_t adc_h = (uint16_t)((b[8] << 8) | b[9]);
    const bool     high  = (s_cal.variant == VARIANT_GAS_HIGH);
    const uint8_t  g_lsb = high ? b[16] : b[14];
    const uint16_t adc_g = (uint16_t)(((high ? b[15] : b[13]) << 2) | (g_lsb >> 6));
    const uint8_t  range = g_lsb & GAS_RANGE_MSK;

    out->temperature_c = comp_temperature(adc_t, &t_fine);
    out->pressure_hpa  = comp_pressure(adc_p, t_fine) / 100.0f;
    out->humidity_pct  = comp_humidity(adc_h, t_fine);
    out->gas_valid     = (g_lsb & GAS_VALID_MSK) && (g_lsb & HEAT_STAB_MSK);
    out->gas_ohm       = high ? comp_gas_high(adc_g, range) : comp_gas_low(adc_g, range);

    s_amb_c = out->temperature_c;
    return ESP_OK;

err:
    ESP_LOGW(TAG, "Leitura falhou depois de %lu ms de espera: %s", (unsigned long)waited, esp_err_to_name(ret));
    return ret;
}
