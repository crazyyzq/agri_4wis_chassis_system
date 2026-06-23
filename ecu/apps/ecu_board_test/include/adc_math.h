#ifndef ECU_ADC_MATH_H
#define ECU_ADC_MATH_H
#include <stdbool.h>
#include <stdint.h>
uint32_t adc_pin_uv_from_external_uv(uint32_t external_uv);
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv);
bool adc_external_in_tolerance(uint32_t measured_uv, uint32_t expected_uv);
#endif
