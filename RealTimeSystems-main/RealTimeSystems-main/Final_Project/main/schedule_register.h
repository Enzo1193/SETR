// schedule_register.h
#ifndef SCHEDULE_REGISTER_H
#define SCHEDULE_REGISTER_H

#include <stdint.h>

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t days[7]; // Array for the days of the week (0 = Sunday, 1 = Monday, ..., 6 = Saturday)
} ScheduleRegister;

void save_schedule_register(uint8_t register_num, ScheduleRegister *register_data);
void load_schedule_register(uint8_t register_num, ScheduleRegister *register_data);

#endif // SCHEDULE_REGISTER_H
