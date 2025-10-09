#include "servo.h"
#include "driver/ledc.h"
#include "esp_err.h"


#define SERVO_PIN 4
#define SERVO_CHANNEL LEDC_CHANNEL_2
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_FREQ 50
#define SERVO_RESOLUTION LEDC_TIMER_8_BIT

void servo_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CHANNEL,
        .timer_sel = SERVO_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void servo_set_angle(uint8_t angle) {
    uint32_t duty = (angle * (1 << SERVO_RESOLUTION)) / 180;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CHANNEL));
}