/* Public entry points registered by test_registry.c. */
#ifndef ECU_TEST_CASES_H
#define ECU_TEST_CASES_H
#include "test_runner.h"
/**
 * @brief Verify that tracked vehicle-affecting resources are safe after boot.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS when outputs, CAN termination, and RS485 TX masks are zero;
 *         otherwise TEST_FAIL.
 */
test_status_t test_safe_boot(test_context_t *context);
/**
 * @brief Validate the 12 V, 5 V, 3.3 V, 1.8 V, and 1.1 V supply rails.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS when all operator-entered millivolt readings are in range,
 *         TEST_FAIL for an out-of-range rail, or TEST_BLOCKED for invalid input.
 */
test_status_t test_power(test_context_t *context);
/**
 * @brief Give the operator direct RGB ownership and verify red, green, and blue.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS when all colors are confirmed; otherwise TEST_FAIL.
 * @note test_rgb_cleanup() must restore RGB status ownership on every exit.
 */
test_status_t test_rgb(test_context_t *context);
/** @brief Turn all RGB channels off and restore status rendering. @param context Unused test context. */
void test_rgb_cleanup(test_context_t *context);
/**
 * @brief Destructively exercise the private 4 KiB internal-RAM scratch buffer.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after all patterns, TEST_BLOCKED on abort, or TEST_FAIL on a
 *         readback mismatch.
 */
test_status_t test_internal_ram(test_context_t *context);
/**
 * @brief Destructively pattern-test the complete 32 MiB external SDRAM.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after all 1 MiB chunks, TEST_BLOCKED when permission is
 *         denied or the operator aborts, or TEST_FAIL on a readback mismatch.
 * @warning All data in BOARD_SDRAM_ADDRESS..BOARD_SDRAM_ADDRESS+32 MiB is lost.
 */
test_status_t test_sdram(test_context_t *context);
/**
 * @brief Erase, program, verify, and re-erase the reserved QSPI test region.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS after verified program/read and final erase, TEST_BLOCKED for
 *         denied permission or an unsafe geometry/range, or TEST_FAIL on ROM I/O.
 * @warning May access only 0x807F0000..0x80800000 and leaves that range erased.
 */
test_status_t test_qspi_flash(test_context_t *context);
/**
 * @brief Verify EEPROM writes while preserving its original test-window bytes.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS only when the pattern verifies and the original 16 bytes are
 *         restored and verified; otherwise TEST_FAIL.
 * @note Saves, overwrites, and restores addresses 0xF0..0xFF in two 8-byte pages.
 */
test_status_t test_eeprom(test_context_t *context);
/**
 * @brief Calibrate-check four external ADC inputs at four prompted voltages.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS when every 128-sample mean is within 100000 uV and monotonic,
 *         TEST_BLOCKED when a fixture step is declined, or TEST_FAIL on ADC/error.
 */
test_status_t test_adc(test_context_t *context);
/**
 * @brief Verify inactive 0 V and active 12 V states on all 12 digital inputs.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS after stable readings, TEST_BLOCKED when a fixture step is
 *         declined, or TEST_FAIL when any 20-sample stability check fails.
 */
test_status_t test_digital_input(test_context_t *context);
/**
 * @brief Pulse and meter-check all 12 protected digital outputs.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS for valid ON/OFF millivolt readings, TEST_BLOCKED for missing
 *         fixture/input, or TEST_FAIL for unsafe ownership or electrical limits.
 * @warning Requires a 220 Ohm / 2 W load; each ON pulse is limited to 500 ms.
 */
test_status_t test_digital_output(test_context_t *context);
/**
 * @brief Verify each digital output through its matching fixture-looped input.
 * @param context Test context; currently unused and may be null.
 * @return TEST_PASS when all pairs assert/release, TEST_BLOCKED when the fixture
 *         is not confirmed, or TEST_FAIL for ownership/readback failure.
 */
test_status_t test_dio_loopback(test_context_t *context);
/** @brief Unconditionally return all managed output resources safe. @param context Unused test context. */
void test_output_cleanup(test_context_t *context);
/**
 * @brief Test four isolated 500 kbit/s Classic CAN channels and termination.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after echo and 115..125 Ohm checks, TEST_BLOCKED for missing
 *         fixture/abort, or TEST_FAIL for protocol, timeout, or meter failure.
 * @note Sends 1000 standard/extended Classic CAN frames per port with DLC 0 or
 *       8. CAN-FD is disabled and is outside test scope.
 */
test_status_t test_can(test_context_t *context);
/** @brief Disable termination on all four CAN channels. @param context Unused test context. */
void test_can_cleanup(test_context_t *context);
/**
 * @brief Echo deterministic frames through three isolated RS485 channels.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after 1000 frames per port, TEST_BLOCKED for missing fixture
 *         or abort, or TEST_FAIL for configuration/echo failure.
 * @note Uses 115200 8N1 and UART hardware-DE pin routing.
 */
test_status_t test_rs485(test_context_t *context);
/** @brief Return all three logical RS485 channels to receive. @param context Unused test context. */
void test_rs485_cleanup(test_context_t *context);
/**
 * @brief Echo deterministic frames through four isolated RS232 channels.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after 1000 frames and polarity confirmation per port,
 *         TEST_BLOCKED for missing fixture/abort, or TEST_FAIL on any mismatch.
 * @note Uses 115200 8N1.
 */
test_status_t test_rs232(test_context_t *context);
/**
 * @brief Decode 100 SBUS frames and verify a commanded silent interval.
 * @param context Active context used for cooperative abort polling.
 * @return TEST_PASS after valid frames and 50 ms silence, TEST_BLOCKED for missing
 *         fixture/abort, or TEST_FAIL for configuration, framing, or stray data.
 * @note Uses 100000 baud, 8 data bits, even parity, and 2 stop bits (8E2).
 */
test_status_t test_sbus(test_context_t *context);
/**
 * @brief Report the intentionally unconnected Ethernet fixture as optional.
 * @param context Test context; currently unused and may be null.
 * @return Always TEST_SKIP because no Ethernet peer is connected for this phase.
 */
test_status_t test_ethernet_skip(test_context_t *context);
#endif
