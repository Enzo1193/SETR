#ifndef SERVO_H
#define SERVO_H

#include "driver/ledc.h"
#include "esp_err.h"

void servo_init(void);
void servo_set_angle(uint8_t angle);

#endif // SERVO_H
