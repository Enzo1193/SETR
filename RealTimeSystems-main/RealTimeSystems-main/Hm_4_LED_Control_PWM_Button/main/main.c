#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "LED_RGB.h"
#include "adc.h"
#include "isr.h"
#include <math.h>
#include "esp_log.h"

// Global variables
QueueHandle_t queue;
LED LED1;


// Function prototypes
void task_range(void *args);
void update_led(ledc_channel_t channel, const char *channel_name, int duty);
void update_duty(LED* led, uint16_t flags, uint16_t duty);
void task_adc(void *args);
void led_init(void);
void process_command(char *command);
void verify_entry(void);

static const char *TAG = "Main";





// Function to update LED duty cycle
void update_led(ledc_channel_t channel, const char *channel_name, int duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    ESP_LOGI(TAG, "UPDATE %s TO: %d", channel_name, duty);
}

// Function to update duty cycle for a specific LED
void update_duty(LED* led, uint16_t flags, uint16_t duty) {
    switch (flags) {
        case 0:
            led->Duty_R = duty;
            update_led(led->CHANEL_R, "RED", led->Duty_R);
            break;
        case 1:
            led->Duty_G = duty;
            update_led(led->CHANEL_G, "GREEN", led->Duty_G);
            break;
        case 2:
            led->Duty_B = duty;
            update_led(led->CHANEL_B, "BLUE", led->Duty_B);
            break;
        case 3:
            ESP_LOGI(TAG, "No LED selected");
            break;
    }
}

// Task to handle ADC readings
void task_adc(void *args) {
    
    while (1) {
        uint16_t duty_value = adc_read_smooth();  
        update_duty(&LED1, flags, duty_value);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Function to initialize LEDs
void led_init(void) {
    LED1 = (LED){
        .CHANEL_R = LEDC_CHANNEL_0, .CHANEL_G = LEDC_CHANNEL_1, .CHANEL_B = LEDC_CHANNEL_2,
        .PIN_R = 5, .PIN_G = 18, .PIN_B = 19,
        .Duty_R = 0, .Duty_G = 0, .Duty_B = 0
    };
 
    configurar_led(&LED1);
    
    ESP_LOGI(TAG, "LED initialized successfully");
}


// Main application function
void app_main(void) {
    isr_install();
    adc_config();
    led_init();
    
 
}