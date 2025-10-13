/*
 * Lectura NTC 10k con divisor (NTC arriba, Rfix=1k abajo) en GPIO34 (ADC1_CH6)
 * Calcula T(°C) usando modelo Beta.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "NTC_TEMP";

/* ---------- Config ADC ---------- */
#define NTC_ADC_CHANNEL     ADC_CHANNEL_6      // GPIO34 -> ADC1_CH6
#define NTC_ADC_ATTEN       ADC_ATTEN_DB_11    // ~0..3.3 V
#define VCC_MV              3300               // Alimentación en mV

/* ---------- Param NTC ---------- */
#define R0_OHM              10000.0f           // 10k @ 25°C
#define T0_K                (25.0f + 273.15f)  // 298.15 K
#define BETA_K              3950.0f            // Beta típico
#define RFIX_OHM            1000.0f            // Resistencia fija 1k (abajo)

/* Prototipos calibración (reutilizamos helpers del ejemplo) */
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void app_main(void)
{
    // --- ADC1 init ---
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    // --- Canal ---
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = NTC_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, NTC_ADC_CHANNEL, &chan_cfg));

    // --- Calibración ---
    adc_cali_handle_t cali = NULL;
    bool do_cali = example_adc_calibration_init(ADC_UNIT_1, NTC_ADC_CHANNEL, NTC_ADC_ATTEN, &cali);

    // Variables de lectura y control de impresiones
    int raw = 0, v_mv = 0;
    float last_temp_c = -1000.0f;             // valor imposible para forzar primera impresión
    const float delta_print_c = 0.5f;         // imprime si cambia más de 0.5 °C
    const TickType_t Ts = pdMS_TO_TICKS(100); // 10 Hz

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, NTC_ADC_CHANNEL, &raw));

        // Convertimos a mV si hay calibración; si no, estimarías V a partir de RAW si lo deseas
        if (do_cali) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, raw, &v_mv));
        } else {
            // Si no hay calibración, puedes dejar raw o implementar un mapeo aproximado si quisieras.
            v_mv = -1; // indicador de no calibrado
        }

        // Si tenemos V en mV, calculamos temperatura
        if (v_mv > 0 && v_mv < VCC_MV) {
            // Divisor con NTC arriba y Rfix abajo:
            // Vout = Vcc * (Rntc / (Rntc + Rfix))  ->  Rntc = Rfix * Vout / (Vcc - Vout)
            float v = (float)v_mv; // mV
            float r_ntc = RFIX_OHM * (v / (float)(VCC_MV - v));

            // Modelo Beta:  1/T = 1/T0 + (1/B)*ln(R/R0)
            float invT = (1.0f / T0_K) + (1.0f / BETA_K) * logf(r_ntc / R0_OHM);
            float T_k = 1.0f / invT;
            float T_c = T_k - 273.15f;

            // Imprimir solo si cambia lo suficiente
            if (fabsf(T_c - last_temp_c) > delta_print_c) {
                ESP_LOGI(TAG, "RAW:%4d  V:%.3f V  Rntc:%.0f ohm  Temp:%.2f C",
                         raw, v / 1000.0f, r_ntc, T_c);
                last_temp_c = T_c;
            }
        } else {
            // Voltaje fuera de rango o sin calibración
            ESP_LOGI(TAG, "RAW:%4d  (sin calibración o fuera de rango)", raw);
        }

        vTaskDelay(Ts);
    }

    // Nunca llegamos aquí en este ejemplo, pero por prolijidad:
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_cali) example_adc_calibration_deinit(cali);
}

/* ------------------ Calibración (helpers del ejemplo) ------------------ */
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_curve_fitting_config_t cfg = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t cfg = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibración OK");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "Sin eFuse de cal.; se omite calibración exacta");
    } else {
        ESP_LOGE(TAG, "Error calibrando");
    }
    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}