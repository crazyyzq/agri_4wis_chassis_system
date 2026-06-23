#include "adc_math.h"
#include "test_limits.h"
uint32_t adc_pin_uv_from_external_uv(uint32_t external_uv)
{
    return (uint32_t)(((uint64_t)external_uv * 100U) / 156U);
}
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv)
{
    uint32_t pin_uv = (uint32_t)(((uint64_t)raw * reference_uv) / 65535U);
    return (uint32_t)(((uint64_t)pin_uv * 156U) / 100U);
}
bool adc_external_in_tolerance(uint32_t measured_uv, uint32_t expected_uv)
{
    uint32_t delta = measured_uv > expected_uv ? measured_uv - expected_uv : expected_uv - measured_uv;
    return delta <= TEST_ADC_TOLERANCE_UV;
}
