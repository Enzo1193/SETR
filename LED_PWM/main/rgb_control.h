#ifndef RGB_CONTROL_H
#define RGB_CONTROL_H

#include <stdint.h>             // 🔹 Asegura definición de uint8_t
#include "driver/ledc.h"
#include "esp_err.h"

// ======================= Function Prototypes =======================

// 🔧 Initializes the PWM channels for both RGB LEDs
void rgb_init(void);

// 🔧 Converts a color value (0–255) into R, G, B components
void rgb_from_value(uint8_t value, uint8_t *r, uint8_t *g, uint8_t *b);

// 🔧 Applies color and intensity (1–100%) to both RGB LEDs
void rgb_set(uint8_t color_value, uint8_t intensity_percent);

void rgb_set_single(uint8_t led_id, uint8_t color_value, uint8_t intensity_percent);


#endif // RGB_CONTROL_H

