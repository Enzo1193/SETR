// main.c  (ESP-IDF)
// Toggle LED blinking each time the BOOT button is pressed.
// Board: ESP32 DevKit V1 (ESP-WROOM-32)
// Button BOOT -> GPIO0 (active LOW).
// LED D2 on-board -> GPIO2.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO        2      // D2 LED on the DevKit V1
#define BUTTON_GPIO     0      // BOOT button on ESP32 DevKit V1
#define BLINK_PERIOD_MS 500    // 500 ms on/off -> 1 Hz total
#define DEBOUNCE_MS     30     // Button debounce time
#define TICK_MS         10     // Main loop tick

static const char *TAG = "BLINK_BTN";

void app_main(void)
{
    // --- Configure LED as output (start OFF) ---
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    // --- Configure BOOT button as input with pull-up ---
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_cfg);

    ESP_LOGI(TAG, "Ready. LED pin=%d (D2), BOOT pin=%d. Press BOOT to start/stop blinking.",
             LED_GPIO, BUTTON_GPIO);

    // --- Simple state machine ---
    bool blinking = false;     // Start stopped
    bool led_state = false;
    int  acc_ms = 0;           // Accumulator to time the blink
    int  last_btn = gpio_get_level(BUTTON_GPIO); // Expect 1 (not pressed)

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        acc_ms += TICK_MS;

        // ---- Debounced button edge: detect press (HIGH->LOW) ----
        int now = gpio_get_level(BUTTON_GPIO);
        if (last_btn == 1 && now == 0) {
            // Possible press -> debounce
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                // Valid press: toggle blinking
                blinking = !blinking;
                ESP_LOGI(TAG, "BOOT pressed -> blinking %s", blinking ? "ON" : "OFF");

                // Wait for release to avoid multiple toggles while held
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                // Release debounce
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            }
        }
        last_btn = now;

        // ---- Blink timing (non-blocking) ----
        if (blinking) {
            if (acc_ms >= BLINK_PERIOD_MS) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state);
                acc_ms = 0;
            }
        } else {
            // Ensure LED is OFF when not blinking
            if (led_state) {
                led_state = false;
                gpio_set_level(LED_GPIO, 0);
            }
            if (acc_ms > BLINK_PERIOD_MS) acc_ms = 0;
        }
    }
}
