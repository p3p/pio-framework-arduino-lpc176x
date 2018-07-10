#pragma once

#include <stdint.h>

void HAL_adc_init(void);
void HAL_adc_enable_channel(int ch);
void HAL_adc_start_conversion(const uint8_t ch);
bool HAL_adc_finished(void);
uint16_t HAL_adc_get_result(void);
void HAL_pwm_init(void);