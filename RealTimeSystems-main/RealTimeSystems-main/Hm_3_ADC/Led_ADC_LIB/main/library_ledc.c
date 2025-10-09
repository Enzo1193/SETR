
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define Maximum_Allowed_Duty 100

typedef struct { //creaciion de estructura LED 
    uint16_t CHANEL_R; //canal de PWM para RED
    uint16_t CHANEL_G; //canal de PWM para GREEN
    uint16_t CHANEL_B; //canal de PWM para BLUE


    uint16_t PIN_R; //pin de salida de PWM para RED
    uint16_t PIN_G; //pin de salida de PWM para GREEN
    uint16_t PIN_B; //pin de salida de PWM para BLUE


    uint16_t Duty_R; //establecer utilidad de salida de PWM para RED
    uint16_t Duty_G; //establecer utilidad de salida de PWM para RED
    uint16_t Duty_B; //establecer utilidad de salida de PWM para RED
} LED;

void configurar_led(LED* led) {
        ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 4000,  
        .clk_cfg          = LEDC_AUTO_CLK
    };
     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = led->CHANEL_R,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led->PIN_R,
        .duty           = led->Duty_R, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        ledc_channel_config_t ledc_channel0 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = led->CHANEL_G,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led->PIN_G,
        .duty           = led->Duty_G, 
        .hpoint         = 0
    };
    
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel0));
        ledc_channel_config_t ledc_channel00 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = led->CHANEL_B,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = led->PIN_B,
        .duty           = led->Duty_B, 
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel00));
}

void change_RGB_LED_DUTY(LED* led , uint16_t duty_R , uint16_t duty_G, uint16_t duty_B, uint16_t duty_R2, uint16_t duty_G2, uint16_t duty_B2) {
    if (duty_R > 100) duty_R = 100;

    if (duty_G > 100) duty_G = 100;

    if (duty_B > 100) duty_B = 100;
    
    //Update the duty cycle of the LED



    uint16_t duty_RED = duty_R * 256 / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_R, duty_RED));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_R));

    uint16_t duty_GREEN = duty_G * 256 / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_G, duty_GREEN));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_G));

    uint16_t duty_BLUE = duty_B * 256 / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_B, duty_BLUE));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_B));


    ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_R, duty_R);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_G, duty_G);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_B, duty_B);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_R);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_G);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, led->CHANEL_B);
}