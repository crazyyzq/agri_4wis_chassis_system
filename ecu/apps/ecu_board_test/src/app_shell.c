/* Foreground CLI dispatch, dependency ordering and board-session aggregation. */
#include <stdio.h>
#include <string.h>
#include "app_shell.h"
#include "cli.h"
#include "operator_io.h"
#include "periodic_tx.h"
#include "result_writer.h"
#include "safety_manager.h"
#include "selftest.h"
#include "status_led.h"
#include "test_registry.h"

#define APP_MAX_TESTS 32U

static test_session_t session;
static test_context_t context;

status_led_state_t app_shell_test_led_state(test_status_t status)
{
    return status == TEST_FAIL ? STATUS_LED_FAILED : STATUS_LED_READY;
}

status_led_state_t app_shell_board_led_state(test_board_status_t status)
{
    return status == TEST_BOARD_FAIL || status == TEST_BOARD_ABORTED
        ? STATUS_LED_FAILED
        : STATUS_LED_READY;
}

/** @brief Print the complete operator command grammar. */
static void print_help(void)
{
    printf("Commands: help | list | board <serial> | run <id> | run all | "
           "status | report | abort | SELFTEST.ALL\n");
}

/** @brief Print registered tests in execution order with release metadata. */
static void print_list(void)
{
    size_t count;
    const test_descriptor_t *tests = test_registry_all(&count);
    for (size_t i = 0U; i < count; ++i) {
        printf("%-22s %s dependency=%s\n", tests[i].id,
               tests[i].requirement == TEST_REQUIRED ? "required" : "optional",
               tests[i].dependency_id != NULL ? tests[i].dependency_id : "-");
    }
}

/**
 * @brief Run, report, and aggregate one already-resolved test descriptor.
 *
 * @param descriptor Borrowed registry descriptor to execute.
 * @return The individual test status returned by the lifecycle runner.
 *
 * @note Clears the previous abort request and brackets execution with TESTING
 *       and persistent outcome indications.
 */
static test_status_t run_descriptor(const test_descriptor_t *descriptor)
{
    context.abort_requested = false;
    /* Registered tests receive exclusive ownership of CAN and serial ports. */
    periodic_tx_suspend();
    status_led_set(STATUS_LED_TESTING);
    test_status_t status = test_runner_execute(descriptor, &context);
    periodic_tx_resume();
    test_result_t result = {
        descriptor->id,
        descriptor->requirement,
        status,
        0U,
        status == TEST_BLOCKED ? "operator/equipment prerequisite not met" : "",
    };
    result_writer_print(&result, session.board_serial);
    test_session_add(&session, descriptor->requirement, status);
    status_led_set(app_shell_test_led_state(status));
    return status;
}

/**
 * @brief Execute the ordered registry as one dependency-aware board session.
 *
 * @note Counts all required descriptors before execution so a partial path can
 *       never report PASS. A case is BLOCKED when its earlier dependency did not
 *       pass. The final path always drives managed hardware safe.
 */
static void run_all(void)
{
    char serial[sizeof(session.board_serial)];
    memcpy(serial, session.board_serial, sizeof(serial));
    serial[sizeof(serial) - 1U] = '\0';
    test_session_init(&session, serial);
    size_t count;
    const test_descriptor_t *tests = test_registry_all(&count);
    test_status_t statuses[APP_MAX_TESTS];
    if (count > APP_MAX_TESTS) {
        safety_all_off();
        status_led_set(STATUS_LED_FAILED);
        printf("BOARD FAIL registry_count=%lu exceeds capacity=%u\n",
               (unsigned long)count, APP_MAX_TESTS);
        return;
    }
    uint16_t expected_required = 0U;
    for (size_t i = 0U; i < count; ++i) {
        if (tests[i].requirement == TEST_REQUIRED) {
            ++expected_required;
        }
    }
    test_session_set_expected_required(&session, expected_required);
    for (size_t i = 0U; i < APP_MAX_TESTS; ++i) {
        statuses[i] = TEST_BLOCKED;
    }
    for (size_t i = 0U; i < count; ++i) {
        bool dependency_ok = true;
        if (tests[i].dependency_id != NULL) {
            dependency_ok = false;
            for (size_t j = 0U; j < i; ++j) {
                if (strcmp(tests[j].id, tests[i].dependency_id) == 0) {
                    dependency_ok = statuses[j] == TEST_PASS;
                    break;
                }
            }
        }
        if (!dependency_ok) {
            test_result_t result = {
                tests[i].id,
                tests[i].requirement,
                TEST_BLOCKED,
                0x0102U,
                "dependency did not pass",
            };
            result_writer_print(&result, session.board_serial);
            test_session_add(&session, tests[i].requirement, TEST_BLOCKED);
        } else {
            statuses[i] = run_descriptor(&tests[i]);
        }
    }
    safety_all_off();
    test_board_status_t board_status = test_session_status(&session);
    status_led_set(app_shell_board_led_state(board_status));
    printf("BOARD SUMMARY %s pass=%u fail=%u skip=%u blocked=%u\n",
           test_board_status_name(board_status), session.pass_count,
           session.fail_count, session.skip_count, session.blocked_count);
}

void app_shell_run(void)
{
    test_session_init(&session, "UNSET");
    print_help();
    for (;;) {
        char line[CLI_LINE_MAX + 1U];
        cli_command_t command;
        printf("ecu-test> ");
        (void)operator_read_line(line, sizeof(line));
        switch (cli_parse(line, &command)) {
        case CLI_HELP:
            print_help();
            break;
        case CLI_LIST:
            print_list();
            break;
        case CLI_BOARD:
            test_session_init(&session, command.argument);
            printf("Board serial set to %s\n", session.board_serial);
            break;
        case CLI_RUN_ONE: {
            const test_descriptor_t *test = test_registry_find(command.argument);
            if (test == NULL) {
                printf("Unknown test: %s\n", command.argument);
            } else {
                (void)run_descriptor(test);
            }
            break;
        }
        case CLI_RUN_ALL:
            run_all();
            break;
        case CLI_STATUS:
        case CLI_REPORT:
            printf("BOARD %s serial=%s pass=%u fail=%u skip=%u blocked=%u\n",
                   test_board_status_name(test_session_status(&session)),
                   session.board_serial, session.pass_count, session.fail_count,
                   session.skip_count, session.blocked_count);
            break;
        case CLI_ABORT:
            context.abort_requested = true;
            safety_all_off();
            session.aborted = true;
            status_led_set(STATUS_LED_FAILED);
            printf("ABORTED: all controlled outputs returned safe\n");
            break;
        case CLI_SELFTEST: {
            periodic_tx_suspend();
            status_led_set(STATUS_LED_BOOTING);
            const safety_hw_ops_t *saved_safety_backend = safety_backend();
            int result = selftest_run_all();
            safety_init(saved_safety_backend);
            status_led_init_default();
            /* selftest_periodic_tx() replaces the backend, so restore rather than resume. */
            periodic_tx_init_default();
            status_led_set(result == 0 ? STATUS_LED_READY : STATUS_LED_FAILED);
            break;
        }
        default:
            printf("Invalid command\n");
            print_help();
            break;
        }
    }
}
