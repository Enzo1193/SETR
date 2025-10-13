/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

const static char *TAG = "ADC_POT";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// Para ESP32, GPIO34 corresponde al canal 6 de ADC1
#define POT_ADC_CHANNEL      ADC_CHANNEL_6
#define POT_ADC_ATTEN        ADC_ATTEN_DB_11   // Permite medir hasta ~3.3V

static int adc_raw;
static int voltage;
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void app_main(void)
{
    //------------- ADC1 Init ---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //------------- ADC1 Config ---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = POT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &config));

    //------------- ADC1 Calibration Init ---------------//
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration = example_adc_calibration_init(ADC_UNIT_1, POT_ADC_CHANNEL, POT_ADC_ATTEN, &adc1_cali_handle);

    //------------- Lectura continua del potenciómetro ---------------//
    int last_voltage = -1; // valor anterior (inicializa fuera del while si prefieres)

while (1) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw));

    if (do_calibration) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));

        // Solo imprimir si cambia más de 50 mV (ajustable)
        if (abs(voltage - last_voltage) > 50) {
            ESP_LOGI(TAG, "Voltaje: %.3f V", voltage / 1000.0);
            last_voltage = voltage;
        }
    } else {
        // Si no hay calibración, comparar el valor RAW directamente
        if (abs(adc_raw - last_voltage) > 50) {
            ESP_LOGI(TAG, "Raw: %d", adc_raw);
            last_voltage = adc_raw;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) calibrated = true;
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibración exitosa");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse no quemado, se omite calibración");
    } else {
        ESP_LOGE(TAG, "Error en calibración");
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
