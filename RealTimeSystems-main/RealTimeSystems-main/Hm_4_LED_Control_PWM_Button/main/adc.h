#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ADC_CHANNEL ADC1_CHANNEL_0
#define ADC_ATTENUATION ADC_ATTEN_DB_12
#define NUM_SAMPLES 100
#define ADC_MAX_VALUE 4095

// Function prototypes
void adc_config(void);
uint16_t adc_read_smooth(void);



#endif // ADC_H