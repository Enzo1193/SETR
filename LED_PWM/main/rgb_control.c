#include "rgb_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <stdio.h>

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY          (5000) // 5 kHz PWM

// --- LED1 ---
#define LED1_R_GPIO 15
#define LED1_G_GPIO 16
#define LED1_B_GPIO 17

// --- LED2 ---
#define LED2_R_GPIO 18
#define LED2_G_GPIO 19
#define LED2_B_GPIO 21

// --- LEDC Channels ---
#define LED1_R_CH LEDC_CHANNEL_0
#define LED1_G_CH LEDC_CHANNEL_1
#define LED1_B_CH LEDC_CHANNEL_2
#define LED2_R_CH LEDC_CHANNEL_3
#define LED2_G_CH LEDC_CHANNEL_4
#define LED2_B_CH LEDC_CHANNEL_5

// ======================= Initialization =======================
void rgb_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch[] = {
        { .channel = LED1_R_CH, .gpio_num = LED1_R_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 },
        { .channel = LED1_G_CH, .gpio_num = LED1_G_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 },
        { .channel = LED1_B_CH, .gpio_num = LED1_B_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 },
        { .channel = LED2_R_CH, .gpio_num = LED2_R_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 },
        { .channel = LED2_G_CH, .gpio_num = LED2_G_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 },
        { .channel = LED2_B_CH, .gpio_num = LED2_B_GPIO, .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER, .duty = 0 }
    };

    for (int i = 0; i < 6; i++) {
        ledc_channel_config(&ch[i]);
    }

    printf("RGB LEDs initialized successfully.\n");
}

// ======================= RGB Conversion =======================
void rgb_from_value(uint8_t value, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region    = value / 43;
    uint8_t remainder = (value - (region * 43)) * 6;
    uint8_t q = (255 * (255 - remainder)) / 255;
    uint8_t t = (255 * remainder) / 255;

    switch (region) {
        case 0: *r = 255; *g = t;   *b = 0;   break;
        case 1: *r = q;   *g = 255; *b = 0;   break;
        case 2: *r = 0;   *g = 255; *b = t;   break;
        case 3: *r = 0;   *g = q;   *b = 255; break;
        case 4: *r = t;   *g = 0;   *b = 255; break;
        default:*r = 255; *g = 0;   *b = q;   break;
    }
}

// ======================= Apply Color & Intensity =======================
void rgb_set(uint8_t color_value, uint8_t intensity_percent)
{
    uint8_t r, g, b;
    rgb_from_value(color_value, &r, &g, &b);

    // Scale RGB values by intensity percentage
    r = (r * intensity_percent) / 100;
    g = (g * intensity_percent) / 100;
    b = (b * intensity_percent) / 100;

    // Apply to both LEDs
    ledc_set_duty(LEDC_MODE, LED1_R_CH, (uint32_t)r);
    ledc_set_duty(LEDC_MODE, LED1_G_CH, (uint32_t)g);
    ledc_set_duty(LEDC_MODE, LED1_B_CH, (uint32_t)b);
    ledc_set_duty(LEDC_MODE, LED2_R_CH, (uint32_t)r);
    ledc_set_duty(LEDC_MODE, LED2_G_CH, (uint32_t)g);
    ledc_set_duty(LEDC_MODE, LED2_B_CH, (uint32_t)b);

    // Update PWM signals
    for (int ch = 0; ch < 6; ch++) {
        ledc_update_duty(LEDC_MODE, ch);
    }

    printf("RGB updated -> Color: %d | Intensity: %d%%\n", color_value, intensity_percent);
}

void rgb_set_single(uint8_t led_id, uint8_t color_value, uint8_t intensity_percent)
{
    uint8_t r, g, b;
    rgb_from_value(color_value, &r, &g, &b);

    r = (r * intensity_percent) / 100;
    g = (g * intensity_percent) / 100;
    b = (b * intensity_percent) / 100;

    if (led_id == 1) {
        ledc_set_duty(LEDC_MODE, LED1_R_CH, r);
        ledc_set_duty(LEDC_MODE, LED1_G_CH, g);
        ledc_set_duty(LEDC_MODE, LED1_B_CH, b);
        ledc_update_duty(LEDC_MODE, LED1_R_CH);
        ledc_update_duty(LEDC_MODE, LED1_G_CH);
        ledc_update_duty(LEDC_MODE, LED1_B_CH);
    } else if (led_id == 2) {
        ledc_set_duty(LEDC_MODE, LED2_R_CH, r);
        ledc_set_duty(LEDC_MODE, LED2_G_CH, g);
        ledc_set_duty(LEDC_MODE, LED2_B_CH, b);
        ledc_update_duty(LEDC_MODE, LED2_R_CH);
        ledc_update_duty(LEDC_MODE, LED2_G_CH);
        ledc_update_duty(LEDC_MODE, LED2_B_CH);
    }
}

