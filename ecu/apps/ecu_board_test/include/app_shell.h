/* Blocking foreground command shell and its test-result indication policy. */
#ifndef ECU_APP_SHELL_H
#define ECU_APP_SHELL_H

#include "status_led.h"
#include "test_types.h"

/**
 * @brief Select the persistent RGB state after one test case completes.
 *
 * @param status Individual test outcome.
 * @return STATUS_LED_FAILED for TEST_FAIL; STATUS_LED_READY for PASS, SKIP,
 *         BLOCKED, or any other value.
 */
status_led_state_t app_shell_test_led_state(test_status_t status);
/**
 * @brief Select the persistent RGB state after a board session completes.
 *
 * @param status Aggregate board-session outcome.
 * @return STATUS_LED_FAILED for FAIL or ABORTED; STATUS_LED_READY for PASS,
 *         INCOMPLETE, or any other value.
 */
status_led_state_t app_shell_board_led_state(test_board_status_t status);

/**
 * @brief Run the blocking foreground UART command shell indefinitely.
 *
 * @note This function never returns. UART input waits call status_led_poll(),
 *       so RGB animation remains active while the shell is idle.
 */
void app_shell_run(void);
#endif
