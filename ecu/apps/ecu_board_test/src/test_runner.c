#include <stddef.h>
#include "board.h"
#include "hpm_uart_drv.h"
#include "safety_manager.h"
#include "status_led.h"
#include "test_runner.h"

bool test_runner_poll_abort(test_context_t *context)
{
    static const char command[] = "abort";
    static uint8_t matched;
    uint8_t byte;
    if (context == NULL) return true;
    status_led_poll();
    while (uart_try_receive_byte(BOARD_CONSOLE_UART_BASE, &byte) == status_success) {
        if (byte == (uint8_t)command[matched]) {
            if (++matched == sizeof(command) - 1U) {
                context->abort_requested = true;
                matched = 0U;
                safety_all_off();
                return true;
            }
        } else {
            matched = byte == (uint8_t)command[0] ? 1U : 0U;
        }
    }
    return context->abort_requested;
}

test_status_t test_runner_execute(const test_descriptor_t *descriptor,
                                  test_context_t *context)
{
    if (descriptor == NULL || context == NULL || descriptor->execute == NULL) return TEST_BLOCKED;
    if (context->abort_requested) return TEST_BLOCKED;

    test_status_t status = TEST_PASS;
    bool prepared = false;
    if (descriptor->prepare != NULL) {
        status = descriptor->prepare(context);
        prepared = status == TEST_PASS;
    } else {
        prepared = true;
    }
    if (prepared) status = context->abort_requested ? TEST_BLOCKED : descriptor->execute(context);

    /* Cleanup is a lifecycle guarantee, not a best-effort convention. */
    if (prepared && descriptor->cleanup != NULL) descriptor->cleanup(context);
    safety_all_off();
    return status;
}
