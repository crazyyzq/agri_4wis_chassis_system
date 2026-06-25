/* Pure fixed-point conversion helpers for the 100k/56k ADC input divider. */
#ifndef ECU_ADC_MATH_H
#define ECU_ADC_MATH_H
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Convert an external connector voltage to the divided ADC-pin voltage.
 *
 * @param external_uv Connector voltage in microvolts.
 * @return ADC-pin voltage in microvolts.
 *
 * @note Uses the board's 100 kOhm / 56 kOhm divider and 64-bit intermediate
 *       arithmetic to avoid overflow during scaling.
 */
uint32_t adc_pin_uv_from_external_uv(uint32_t external_uv);
/**
 * @brief Convert a 16-bit ADC result to the external connector voltage.
 *
 * @param raw          ADC code in the inclusive range 0..65535.
 * @param reference_uv ADC reference voltage in microvolts.
 * @return Estimated connector voltage in microvolts.
 *
 * @note Uses the board's 100 kOhm / 56 kOhm divider and 64-bit intermediate
 *       arithmetic. Integer division truncates toward zero.
 */
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv);
/**
 * @brief Check a connector-voltage measurement against its expected value.
 *
 * @param measured_uv Measured connector voltage in microvolts.
 * @param expected_uv Expected connector voltage in microvolts.
 * @return true when the absolute error is no greater than
 *         TEST_ADC_TOLERANCE_UV; otherwise false.
 */
bool adc_external_in_tolerance(uint32_t measured_uv, uint32_t expected_uv);
#endif
