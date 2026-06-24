#include <string.h>
#include "selftest.h"
#include "app_shell.h"
#include "cli.h"
#include "test_runner.h"

static uint32_t cleanup_calls;
static test_status_t fake_prepare(test_context_t *ctx) { (void)ctx; return TEST_PASS; }
static test_status_t fake_fail(test_context_t *ctx) { (void)ctx; return TEST_FAIL; }
static void fake_cleanup(test_context_t *ctx) { (void)ctx; ++cleanup_calls; }

bool selftest_cli_runner(void)
{
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

    const test_descriptor_t descriptor = {
        "FAKE.FAIL", TEST_REQUIRED, NULL, 100U,
        fake_prepare, fake_fail, fake_cleanup
    };
    test_context_t context = { 0 };
    cleanup_calls = 0U;
    SELFTEST_ASSERT_EQ(TEST_FAIL, test_runner_execute(&descriptor, &context));
    SELFTEST_ASSERT_EQ(1U, cleanup_calls);
    return true;
}
