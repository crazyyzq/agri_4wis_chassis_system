/* Parser, LED outcome policy, abort recognizer and runner lifecycle checks. */
#include <string.h>
#include "selftest.h"
#include "app_shell.h"
#include "cli.h"
#include "safety_manager.h"
#include "test_runner.h"

static uint32_t cleanup_calls;
static uint32_t execute_calls;
/* No-op callbacks provide a complete backend without touching target hardware. */
static void runner_output_write(uint8_t index, bool on) { (void)index; (void)on; }
static void runner_term_write(uint8_t index, bool on) { (void)index; (void)on; }
static void runner_rs485_write(uint8_t index, bool transmit) { (void)index; (void)transmit; }
static const safety_hw_ops_t runner_safety_ops = {
    runner_output_write, runner_term_write, runner_rs485_write
};
/** @brief Model successful resource acquisition for lifecycle assertions. */
static test_status_t fake_prepare(test_context_t *ctx) { (void)ctx; return TEST_PASS; }
/** @brief Model preparation failure, for which cleanup must not run. */
static test_status_t fake_prepare_fail(test_context_t *ctx) { (void)ctx; return TEST_FAIL; }
/** @brief Count execution and return a deterministic functional failure. */
static test_status_t fake_fail(test_context_t *ctx) { (void)ctx; ++execute_calls; return TEST_FAIL; }
/** @brief Count cleanup calls so lifecycle guarantees are directly asserted. */
static void fake_cleanup(test_context_t *ctx) { (void)ctx; ++cleanup_calls; }

bool selftest_cli_runner(void)
{
    /* Keep this case independent of self-test registration order. */
    safety_init(&runner_safety_ops);
    cli_command_t cmd;
    SELFTEST_ASSERT_EQ(CLI_LIST, cli_parse("list", &cmd));
    SELFTEST_ASSERT_EQ(CLI_RUN_ONE, cli_parse("run ADC.EX1", &cmd));
    SELFTEST_ASSERT_TRUE(strcmp(cmd.argument, "ADC.EX1") == 0);
    SELFTEST_ASSERT_EQ(CLI_RUN_ALL, cli_parse("run all", &cmd));
    SELFTEST_ASSERT_EQ(CLI_BOARD, cli_parse("board ECU-001", &cmd));
    SELFTEST_ASSERT_EQ(CLI_INVALID, cli_parse("run", &cmd));

    SELFTEST_ASSERT_EQ(STATUS_LED_READY, app_shell_test_led_state(TEST_PASS));
    SELFTEST_ASSERT_EQ(STATUS_LED_FAILED, app_shell_test_led_state(TEST_FAIL));
    SELFTEST_ASSERT_EQ(STATUS_LED_READY, app_shell_test_led_state(TEST_SKIP));
    SELFTEST_ASSERT_EQ(STATUS_LED_READY, app_shell_test_led_state(TEST_BLOCKED));
    SELFTEST_ASSERT_EQ(STATUS_LED_READY, app_shell_board_led_state(TEST_BOARD_PASS));
    SELFTEST_ASSERT_EQ(STATUS_LED_FAILED, app_shell_board_led_state(TEST_BOARD_FAIL));
    SELFTEST_ASSERT_EQ(STATUS_LED_READY, app_shell_board_led_state(TEST_BOARD_INCOMPLETE));
    SELFTEST_ASSERT_EQ(STATUS_LED_FAILED, app_shell_board_led_state(TEST_BOARD_ABORTED));

    test_context_t partial = { 0 };
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&partial, (uint8_t)'a'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&partial, (uint8_t)'b'));
    test_context_t independent = { 0 };
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'o'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'r'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'t'));
    SELFTEST_ASSERT_TRUE(!independent.abort_requested);
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'a'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'b'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'o'));
    SELFTEST_ASSERT_TRUE(!test_runner_consume_abort_byte(&independent, (uint8_t)'r'));
    SELFTEST_ASSERT_TRUE(test_runner_consume_abort_byte(&independent, (uint8_t)'t'));
    SELFTEST_ASSERT_TRUE(independent.abort_requested);

    const test_descriptor_t descriptor = {
        "FAKE.FAIL", TEST_REQUIRED, NULL, 100U,
        fake_prepare, fake_fail, fake_cleanup
    };
    test_context_t context = { 0 };
    cleanup_calls = 0U;
    execute_calls = 0U;
    SELFTEST_ASSERT_EQ(TEST_FAIL, test_runner_execute(&descriptor, &context));
    SELFTEST_ASSERT_EQ(1U, cleanup_calls);
    SELFTEST_ASSERT_EQ(1U, execute_calls);

    const test_descriptor_t prepare_failure = {
        "FAKE.PREPARE.FAIL", TEST_REQUIRED, NULL, 100U,
        fake_prepare_fail, fake_fail, fake_cleanup
    };
    cleanup_calls = 0U;
    execute_calls = 0U;
    SELFTEST_ASSERT_EQ(TEST_FAIL, test_runner_execute(&prepare_failure, &context));
    SELFTEST_ASSERT_EQ(0U, cleanup_calls);
    SELFTEST_ASSERT_EQ(0U, execute_calls);

    SELFTEST_ASSERT_EQ(SAFETY_OK, safety_output_on(1U));
    SELFTEST_ASSERT_EQ(TEST_BLOCKED, test_runner_execute(NULL, &context));
    SELFTEST_ASSERT_EQ(0U, safety_snapshot().active_output);
    return true;
}
