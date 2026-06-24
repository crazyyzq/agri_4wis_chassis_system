/* Pure fixed-point conversion helpers for the 100k/56k ADC input divider. */
#ifndef ECU_ADC_MATH_H
#define ECU_ADC_MATH_H
#include <stdbool.h>
#include <stdint.h>
/* Convert external connector voltage to ADC pin voltage, both in microvolts. */
uint32_t adc_pin_uv_from_external_uv(uint32_t external_uv);
/* Convert a 16-bit ADC code to connector voltage in microvolts. */
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv);
/* Compare connector voltage using TEST_ADC_TOLERANCE_UV absolute tolerance. */
bool adc_external_in_tolerance(uint32_t measured_uv, uint32_t expected_uv);
#endif
