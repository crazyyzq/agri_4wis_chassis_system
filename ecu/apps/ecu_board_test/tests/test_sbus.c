/* Verify the interrupt-driven SBUS service receives frames and detects silence. */
#include <stdio.h>
#include "board.h"
#include "operator_io.h"
#include "sbus_service.h"
#include "test_cases.h"

/**
 * @brief Wait for the SBUS service to mark the receiver disconnected.
 *
 * The service owns UART1 continuously, so this test observes its published
 * connection state instead of reading UART bytes directly.
 */
static bool wait_sbus_disconnected(test_context_t *context, uint32_t timeout_ms)
{
    for (uint32_t elapsed = 0U; elapsed < timeout_ms; ++elapsed) {
        if (test_runner_poll_abort(context)) return false;
        sbus_debug_state_t state;
        sbus_service_get_snapshot(&state);
        if (state.connected == 0U) return true;
        board_delay_ms(1U);
    }
    return false;
}

test_status_t test_sbus(test_context_t *context)
{
    if (!operator_confirm("SBUS receiver is connected and transmitting")) return TEST_BLOCKED;
    sbus_debug_state_t start;
    sbus_debug_state_t state;
    sbus_service_get_snapshot(&start);
    uint32_t target = start.frame_count + 100U;
    bool printed_first = false;
    bool printed_last = false;

    for (uint32_t wait = 0U; wait < 3000U && !printed_last; ++wait) {
        if (test_runner_poll_abort(context)) return TEST_BLOCKED;
        sbus_service_get_snapshot(&state);
        if (!printed_first && state.frame_count > start.frame_count) {
            printf("SBUS frame=1 ch1=%u connected=%lu lost=%u failsafe=%u\n",
                   state.channels[0], (unsigned long)state.connected,
                   state.frame_lost, state.failsafe);
            printed_first = true;
        }
        if (state.frame_count >= target) {
            printf("SBUS frame=100 ch1=%u connected=%lu lost=%u failsafe=%u\n",
                   state.channels[0], (unsigned long)state.connected,
                   state.frame_lost, state.failsafe);
            printed_last = true;
        }
        board_delay_ms(1U);
    }
    if (!printed_last) return TEST_FAIL;
    if (!operator_confirm("Stop the SBUS transmitter, then confirm")) return TEST_BLOCKED;
    return wait_sbus_disconnected(context, 300U) ? TEST_PASS :
        (context->abort_requested ? TEST_BLOCKED : TEST_FAIL);
}
