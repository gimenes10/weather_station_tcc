#include "mq_sensor.h"
#include "pin_config.h"
#include "adc_reader.h"
#include "esp_log.h"

static const char *TAG = "mq_sensor";

/* ADC 12-bit com 12dB atten: range ~0..3.1V */
#define ADC_MAX_RAW    4095
#define ADC_REF_MV     3100.0f

/*
 * Compensação do divisor resistivo (12kΩ + 20kΩ) no AOUT dos módulos MQ.
 * O divisor reduz a tensão por fator 20k/(12k+20k) = 5/8.
 * Para obter a tensão real no AOUT, multiplicamos a leitura por 8/5.
 * Saída máxima com 5V: 5 × 5/8 = 3.125V (abaixo do limite de 3.6V do S3;
 * os últimos ~25 mV ficam acima do fundo de escala de ~3.1V do ADC).
 *
 *   AOUT (MQ, até 5V) ──[12kΩ]──┬── GPIO (ESP32, max 3.3V)
 *                                │
 *                              [20kΩ]   [100nF]
 *                                │         │
 *                               GND ──────GND
 */
#define MQ_DIVIDER_FACTOR  1.6f     /* (12k + 20k) / 20k = 8/5 */

/*
 * Teste de presença (adc_reader_probe), com pulls internos de ~45 kΩ:
 *   MQ conectado   : Rth ≈ (12k + RL) ∥ 20k ≈ 7,8 kΩ → swing ≈ 0,4–0,7 V
 *   MQ desconectado: só os 20 kΩ do divisor   → swing ≈ 0,8–1,3 V
 *   Pino solto     : sem divisor               → swing ≈ 3 V
 * A faixa varia com a tolerância do pull interno (30–60 kΩ). Ajustar
 * o limiar pelo swing que aparece no log, conectado e desconectado.
 */
#define MQ_PROBE_MAX_SWING_MV  750

esp_err_t mq_sensor_read(adc_oneshot_unit_handle_t adc, mq_data_t *data)
{
    esp_err_t ret;

    ret = adc_reader_probe(adc, MQ7_ADC_CHANNEL, MQ_PROBE_MAX_SWING_MV,
                           &data->mq7_present, &data->mq7_swing_mv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no teste de presenca do MQ-7: %s", esp_err_to_name(ret));
        goto err_read;
    }

    ret = adc_reader_probe(adc, MQ2_ADC_CHANNEL, MQ_PROBE_MAX_SWING_MV,
                           &data->mq2_present, &data->mq2_swing_mv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha no teste de presenca do MQ-2: %s", esp_err_to_name(ret));
        goto err_read;
    }

    ret = adc_reader_read_raw(adc, MQ7_ADC_CHANNEL, &data->mq7_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler MQ-7: %s", esp_err_to_name(ret));
        goto err_read;
    }

    ret = adc_reader_read_raw(adc, MQ2_ADC_CHANNEL, &data->mq2_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao ler MQ-2: %s", esp_err_to_name(ret));
        goto err_read;
    }

    /* Tensão real no AOUT = tensão no ADC × fator do divisor */
    float adc_mq7_v = ((float)data->mq7_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;
    float adc_mq2_v = ((float)data->mq2_raw / ADC_MAX_RAW) * ADC_REF_MV / 1000.0f;

    data->mq7_voltage = adc_mq7_v * MQ_DIVIDER_FACTOR;
    data->mq2_voltage = adc_mq2_v * MQ_DIVIDER_FACTOR;

    return ESP_OK;

err_read:
    return ret;
}