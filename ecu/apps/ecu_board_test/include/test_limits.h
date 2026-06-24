/* Approved electrical limits, durations and communication test counts. */
#ifndef ECU_TEST_LIMITS_H
#define ECU_TEST_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#define TEST_5V_MIN_MV 4750U
#define TEST_5V_MAX_MV 5250U
#define TEST_3V3_MIN_MV 3135U
#define TEST_3V3_MAX_MV 3465U
#define TEST_BOOT_LOG_DEADLINE_MS 3000U
#define TEST_DO_MAX_ON_MS 500U
#define TEST_COMM_FRAME_COUNT 1000U
#define TEST_ADC_EXTERNAL_FS_UV 5000000U
#define TEST_ADC_TOLERANCE_UV 100000U

/**
 * @brief Check the measured 5 V rail against its inclusive release limits.
 * @param mv Measured voltage in millivolts.
 * @return true for 4750..5250 mV; otherwise false.
 */
static inline bool test_5v_in_range(uint32_t mv) { return mv >= TEST_5V_MIN_MV && mv <= TEST_5V_MAX_MV; }
/**
 * @brief Check the measured 3.3 V rail against its inclusive release limits.
 * @param mv Measured voltage in millivolts.
 * @return true for 3135..3465 mV; otherwise false.
 */
static inline bool test_3v3_in_range(uint32_t mv) { return mv >= TEST_3V3_MIN_MV && mv <= TEST_3V3_MAX_MV; }

#endif
