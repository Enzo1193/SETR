#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO 5
#define BLINK_GPIO_4 4

void gpio_blink_4(void *pvParameter) {
    gpio_pad_select_gpio(BLINK_GPIO_4);
    gpio_set_direction(BLINK_GPIO_4, GPIO_MODE_OUTPUT);

    while (1) {
        for (uint8_t i = 0; i < 100; i++) {
            // Blink off (output low)
            gpio_set_level(BLINK_GPIO_4, 0);
            ets_Delay_us(150); 
            // Blink on (output high)
            gpio_set_level(BLINK_GPIO_4, 1);
            ets_Delay_us(150); 
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }
}

void blink_5(void *pvParameter) {
    // Configure GPIO5 as output
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink on (output high)
        for (uint8_t i = 0; i < 100; i++) {
            gpio_set_level(BLINK_GPIO, 0);
            ets_Delay_us(150); 
            gpio_set_level(BLINK_GPIO, 1);
            ets_Delay_us(150); 
        }
        vTaskDelay(1);  
    }
}

void app_main() {
    xTaskCreatePinnedToCore(gpio_blink_4, "blink_4", 2048, NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(blink_5, "blink_5", 2048, NULL, 5, NULL,1);
}