#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "nvs_flash.h"
#include "nvs.h"

// ---------- Project pins ----------
#define PIN_LED_R 4
#define PIN_LED_G 5
#define PIN_LED_B 6
#define PIN_POT_ADC_GPIO 0
#define PIN_NTC_ADC_GPIO 1
#define PIN_BOOT_BTN 9

// ---------- NTC constants ----------
#define V_SUPPLY_MV 3300
#define R_SERIES_OHM 1000.0f
#define NTC_R25_OHM 10000.0f
#define NTC_BETA 3950.0f
#define T0_K 298.15f

static const char *TAG = "SETR_MAIN";

// ---------- Structs ----------
typedef struct {
    float temp_c;
    uint16_t pot_mv;
    uint16_t ntc_mv;
} adc_sample_t;

typedef struct {
    bool print_toggle;
} button_event_t;

static QueueHandle_t q_adc = NULL;
static QueueHandle_t q_button = NULL;

// ---------- Globals ----------
static volatile float g_last_temp_c = 25.0f;
static volatile uint16_t g_last_pot_mv = 0;
static volatile uint16_t g_last_ntc_mv = 0;

typedef struct {
    float rmin, gmin, bmin;
} thresholds_t;

static thresholds_t thresholds = {28.0f, 20.0f, 0.0f};

// ---------- NVS ----------
static esp_err_t nvs_init_and_prepare(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static void thresholds_load_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("setr", NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(float);
    float v;
    if (nvs_get_blob(h, "rmin", &v, &sz) == ESP_OK) thresholds.rmin = v;
    sz = sizeof(float);
    if (nvs_get_blob(h, "gmin", &v, &sz) == ESP_OK) thresholds.gmin = v;
    sz = sizeof(float);
    if (nvs_get_blob(h, "bmin", &v, &sz) == ESP_OK) thresholds.bmin = v;
    nvs_close(h);
}

static void thresholds_save_to_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open("setr", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "rmin", &thresholds.rmin, sizeof(float));
    nvs_set_blob(h, "gmin", &thresholds.gmin, sizeof(float));
    nvs_set_blob(h, "bmin", &thresholds.bmin, sizeof(float));
    nvs_commit(h);
    nvs_close(h);
}

// ---------- Helpers ----------
static void init_usb_serial(void)
{
    usb_serial_jtag_driver_config_t cfg = {.tx_buffer_size = 256, .rx_buffer_size = 256};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

static void init_boot_button(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&io));
}

static void init_ledc_pwm(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    const int pins[3] = {PIN_LED_R, PIN_LED_G, PIN_LED_B};
    for (int i = 0; i < 3; i++) {
        ledc_channel_config_t c = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

// ---------- ADC helpers ----------
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_pot = NULL, cali_ntc = NULL;

static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_curve_fitting_config_t cfg = {.unit_id = unit, .atten = atten, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_cali_handle_t handle = NULL;
    if (adc_cali_create_scheme_curve_fitting(&cfg, &handle) == ESP_OK) {
        *out_handle = handle;
        return true;
    }
    return false;
}

static float ntc_resistance_from_mv(uint16_t mv)
{
    float v = (float)mv;
    if (v <= 0.0f)
        v = 0.1f;
    if (v >= V_SUPPLY_MV - 0.1f)
        v = V_SUPPLY_MV - 0.1f;
    return R_SERIES_OHM * v / (V_SUPPLY_MV - v);
}

static float ntc_temp_c_from_resistance(float r)
{
    if (r <= 1.0f)
        r = 1.0f;
    float invT = (1.0f / T0_K) + (1.0f / NTC_BETA) * logf(r / NTC_R25_OHM);
    return (1.0f / invT) - 273.15f;
}

// ---------- Tasks ----------
static void task_adc_read(void *arg)
{
    // Solo lectura ADC
    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));
    adc_oneshot_chan_cfg_t ch_cfg = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &ch_cfg));

    bool cal_pot = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &cali_pot);
    bool cal_ntc = adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &cali_ntc);

    while (1) {
        int raw_pot, raw_ntc, mv_pot, mv_ntc;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw_pot);
        adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &raw_ntc);
        if (cal_pot)
            adc_cali_raw_to_voltage(cali_pot, raw_pot, &mv_pot);
        else
            mv_pot = raw_pot;
        if (cal_ntc)
            adc_cali_raw_to_voltage(cali_ntc, raw_ntc, &mv_ntc);
        else
            mv_ntc = raw_ntc;

        float rntc = ntc_resistance_from_mv(mv_ntc);
        float temp_c = ntc_temp_c_from_resistance(rntc);

        g_last_temp_c = temp_c;
        g_last_pot_mv = mv_pot;
        g_last_ntc_mv = mv_ntc;

        adc_sample_t s = {.temp_c = temp_c, .pot_mv = mv_pot, .ntc_mv = mv_ntc};
        xQueueSend(q_adc, &s, 0);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void task_telemetry(void *arg)
{
    while (1) {
        char line[96];
        int n = snprintf(line, sizeof(line),
                         "POT:%4u mV | NTC:%4u mV | Rntc:%6.0f ohm | T:%6.2f C\r\n",
                         (unsigned)g_last_pot_mv,
                         (unsigned)g_last_ntc_mv,
                         (double)ntc_resistance_from_mv(g_last_ntc_mv),
                         (double)g_last_temp_c);
        usb_serial_jtag_write_bytes(line, n, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void task_temp_print(void *arg)
{
    bool enabled = true;
    button_event_t ev;
    while (1) {
        if (xQueueReceive(q_button, &ev, 0)) {
            enabled = ev.print_toggle;
            const char *msg = enabled ?
                "[INFO] Temperature printing ENABLED\r\n" :
                "[INFO] Temperature printing DISABLED\r\n";
            usb_serial_jtag_write_bytes(msg, strlen(msg), portMAX_DELAY);
        }

        if (enabled) {
            char msg[64];
            int n = snprintf(msg, sizeof(msg), "Temp: %.2f C\r\n", (double)g_last_temp_c);
            usb_serial_jtag_write_bytes(msg, n, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // cada 2 s
    }
}

static void task_button(void *arg)
{
    bool print_flag = true;
    int last = gpio_get_level(PIN_BOOT_BTN);
    while (1) {
        int now = gpio_get_level(PIN_BOOT_BTN);
        if (last == 1 && now == 0) {
            print_flag = !print_flag;
            button_event_t ev = {.print_toggle = print_flag};
            xQueueSend(q_button, &ev, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        last = now;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void task_rgb_control(void *arg)
{
    adc_sample_t rx;
    while (1) {
        if (xQueueReceive(q_adc, &rx, portMAX_DELAY)) {
            float brightness = rx.pot_mv / 3300.0f;
            brightness = fminf(fmaxf(brightness, 0.0f), 1.0f);
            float r = (rx.temp_c >= thresholds.rmin);
            float g = (rx.temp_c >= thresholds.gmin);
            float b = (rx.temp_c >= thresholds.bmin);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r * brightness * 4095);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g * brightness * 4095);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b * brightness * 4095);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
        }
    }
}

static void task_usb_uart(void *arg)
{
    const char *hello = "[SETR] USB console ready.\r\nCommands: rmin <T>, gmin <T>, bmin <T>, get\r\n";
    usb_serial_jtag_write_bytes(hello, strlen(hello), portMAX_DELAY);
    uint8_t buf[64];
    char line[64];
    int idx = 0;
    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\r' || c == '\n') {
                line[idx] = '\0';
                idx = 0;
                if (line[0] == '\0') continue;
                float val;
                if (sscanf(line, "rmin %f", &val) == 1) thresholds.rmin = val, thresholds_save_to_nvs();
                else if (sscanf(line, "gmin %f", &val) == 1) thresholds.gmin = val, thresholds_save_to_nvs();
                else if (sscanf(line, "bmin %f", &val) == 1) thresholds.bmin = val, thresholds_save_to_nvs();
                else if (strcmp(line, "get") == 0) {
                    char msg[80];
                    int m = snprintf(msg, sizeof(msg), "[THR] R>=%.2f | G>=%.2f | B>=%.2f\r\n",
                                     thresholds.rmin, thresholds.gmin, thresholds.bmin);
                    usb_serial_jtag_write_bytes(msg, m, portMAX_DELAY);
                }
            } else if (idx < sizeof(line) - 1)
                line[idx++] = c;
        }
    }
}

// ---------- app_main ----------
void app_main(void)
{
    nvs_init_and_prepare();
    thresholds_load_from_nvs();
    init_usb_serial();
    init_ledc_pwm();
    init_boot_button();

    q_adc = xQueueCreate(8, sizeof(adc_sample_t));
    q_button = xQueueCreate(4, sizeof(button_event_t));

    xTaskCreate(task_adc_read, "adc", 4096, NULL, 5, NULL);
    xTaskCreate(task_telemetry, "telemetry", 3072, NULL, 2, NULL);
    xTaskCreate(task_temp_print, "temp", 4096, NULL, 3, NULL);
    xTaskCreate(task_button, "button", 3072, NULL, 4, NULL);
    xTaskCreate(task_rgb_control, "rgb", 4096, NULL, 4, NULL);
    xTaskCreate(task_usb_uart, "usb", 4096, NULL, 5, NULL);
}