/* Enforce test lifecycle cleanup and recognize cooperative UART abort input. */
#include <stddef.h>
#include "board.h"
#include "hpm_uart_drv.h"
#include "safety_manager.h"
#include "sbus_service.h"
#include "status_led.h"
#include "test_runner.h"

bool test_runner_poll_abort(test_context_t *context)
{
    uint8_t byte;
    if (context == NULL) {
        safety_all_off();
        return true;
    }
    status_led_poll();
    sbus_service_poll();
    /* Drain without waiting so long hardware loops can call this frequently. */
    while (uart_try_receive_byte(BOARD_CONSOLE_UART_BASE, &byte) == status_success) {
        if (test_runner_consume_abort_byte(context, byte)) {
            safety_all_off();
            return true;
        }
    }
    return context->abort_requested;
}

bool test_runner_consume_abort_byte(test_context_t *context, uint8_t byte)
{
    /* Deliberately case-sensitive: only the exact lowercase safety word acts. */
    static const uint8_t command[] = { 'a', 'b', 'o', 'r', 't' };
    if (context == NULL) {
        return true;
    }
    if (context->abort_requested) {
        return true;
    }
    if (context->abort_match_length >= sizeof(command)) {
        context->abort_match_length = 0U;
    }

    if (byte == command[context->abort_match_length]) {
        ++context->abort_match_length;
        if (context->abort_match_length == sizeof(command)) {
            context->abort_requested = true;
            context->abort_match_length = 0U;
            return true;
        }
    } else {
        context->abort_match_length = byte == command[0] ? 1U : 0U;
    }
    return false;
}

test_status_t test_runner_execute(const test_descriptor_t *descriptor,
                                  test_context_t *context)
{
    if (descriptor == NULL || context == NULL || descriptor->execute == NULL) {
        safety_all_off();
        return TEST_BLOCKED;
    }
    context->abort_match_length = 0U;
    if (context->abort_requested) {
        safety_all_off();
        return TEST_BLOCKED;
    }

    test_status_t status = TEST_PASS;
    bool prepared = false;
    if (descriptor->prepare != NULL) {
        status = descriptor->prepare(context);
        prepared = status == TEST_PASS;
    } else {
        prepared = true;
    }
    if (prepared) status = context->abort_requested ? TEST_BLOCKED : descriptor->execute(context);

    /* A failed prepare owns no acquired fixture state, so its cleanup is skipped. */
    if (prepared && descriptor->cleanup != NULL) descriptor->cleanup(context);
    /* Safe shutdown is unconditional, including invalid and prepare-failure paths. */
    safety_all_off();
    return status;
}
