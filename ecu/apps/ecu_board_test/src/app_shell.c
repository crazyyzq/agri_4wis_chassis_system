#include <stdio.h>
#include <string.h>
#include "app_shell.h"
#include "cli.h"
#include "operator_io.h"
#include "result_writer.h"
#include "safety_manager.h"
#include "selftest.h"
#include "status_led.h"
#include "test_registry.h"
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

static void print_help(void) { printf("Commands: help | list | board <serial> | run <id> | run all | status | report | abort | SELFTEST.ALL\n"); }
static void print_list(void)
{
    size_t count; const test_descriptor_t *tests = test_registry_all(&count);
    for (size_t i = 0U; i < count; ++i) printf("%-22s %s dependency=%s\n", tests[i].id,
        tests[i].requirement == TEST_REQUIRED ? "required" : "optional",
        tests[i].dependency_id != NULL ? tests[i].dependency_id : "-");
}
static test_status_t run_descriptor(const test_descriptor_t *descriptor)
{
    context.abort_requested = false;
    status_led_set(STATUS_LED_TESTING);
    test_status_t status = test_runner_execute(descriptor, &context);
    test_result_t result = { descriptor->id, descriptor->requirement, status, 0U,
        status == TEST_BLOCKED ? "operator/equipment prerequisite not met" : "" };
    result_writer_print(&result, session.board_serial);
    test_session_add(&session, descriptor->requirement, status);
    status_led_set(app_shell_test_led_state(status));
    return status;
}
static void run_all(void)
{
    char serial[sizeof(session.board_serial)];
    memcpy(serial, session.board_serial, sizeof(serial));
    serial[sizeof(serial) - 1U] = '\0';
    test_session_init(&session, serial);
    size_t count; const test_descriptor_t *tests = test_registry_all(&count);
    test_status_t statuses[32];
    for (size_t i = 0U; i < 32U; ++i) statuses[i] = TEST_BLOCKED;
    for (size_t i = 0U; i < count; ++i) {
        bool dependency_ok = true;
        if (tests[i].dependency_id != NULL) {
            dependency_ok = false;
            for (size_t j = 0U; j < i; ++j)
                if (strcmp(tests[j].id, tests[i].dependency_id) == 0) dependency_ok = statuses[j] == TEST_PASS;
        }
        if (!dependency_ok) {
            test_result_t result = { tests[i].id, tests[i].requirement, TEST_BLOCKED, 0x0102U, "dependency did not pass" };
            result_writer_print(&result, session.board_serial);
            test_session_add(&session, tests[i].requirement, TEST_BLOCKED);
        } else statuses[i] = run_descriptor(&tests[i]);
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
    test_session_init(&session, "UNSET"); print_help();
    for (;;) {
        char line[CLI_LINE_MAX + 1U]; cli_command_t command;
        printf("ecu-test> "); operator_read_line(line, sizeof(line));
        switch (cli_parse(line, &command)) {
        case CLI_HELP: print_help(); break;
        case CLI_LIST: print_list(); break;
        case CLI_BOARD: test_session_init(&session, command.argument); printf("Board serial set to %s\n", session.board_serial); break;
        case CLI_RUN_ONE: { const test_descriptor_t *test = test_registry_find(command.argument); if (test == NULL) printf("Unknown test: %s\n", command.argument); else run_descriptor(test); break; }
        case CLI_RUN_ALL: run_all(); break;
        case CLI_STATUS:
        case CLI_REPORT: printf("BOARD %s serial=%s pass=%u fail=%u skip=%u blocked=%u\n",
            test_board_status_name(test_session_status(&session)), session.board_serial,
            session.pass_count, session.fail_count, session.skip_count, session.blocked_count); break;
        case CLI_ABORT: context.abort_requested = true; safety_all_off(); session.aborted = true; status_led_set(STATUS_LED_FAILED); printf("ABORTED: all controlled outputs returned safe\n"); break;
        case CLI_SELFTEST: {
            status_led_set(STATUS_LED_BOOTING);
            int result = selftest_run_all();
            status_led_init_default();
            status_led_set(result == 0 ? STATUS_LED_READY : STATUS_LED_FAILED);
            break;
        }
        default: printf("Invalid command\n"); print_help(); break;
        }
    }
}
