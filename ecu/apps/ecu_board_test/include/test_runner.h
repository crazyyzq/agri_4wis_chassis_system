/* Prepare/execute/cleanup lifecycle and cooperative console abort handling. */
#ifndef ECU_TEST_RUNNER_H
#define ECU_TEST_RUNNER_H

#include <stdbool.h>
#include <stdint.h>
#include "test_types.h"

typedef struct {
    volatile bool abort_requested;
    /* Partial foreground-console match; scoped to one test execution. */
    uint8_t abort_match_length;
    void *user;
} test_context_t;

typedef struct test_descriptor {
    const char *id;
    test_requirement_t requirement;
    const char *dependency_id;
    uint32_t timeout_ms;
    test_status_t (*prepare)(test_context_t *context);
    test_status_t (*execute)(test_context_t *context);
    void (*cleanup)(test_context_t *context);
} test_descriptor_t;

/**
 * @brief Execute one test descriptor through prepare, execute, and cleanup.
 *
 * @param descriptor Immutable lifecycle descriptor with a valid execute callback.
 * @param context    Per-execution context carrying abort state and user storage.
 * @return Lifecycle result, or TEST_BLOCKED for invalid input or a preexisting
 *         abort request.
 *
 * @note Cleanup runs only after successful preparation, including when execute
 *       fails. safety_all_off() runs on every exit, including prepare failure.
 */
test_status_t test_runner_execute(const test_descriptor_t *descriptor,
                                  test_context_t *context);
/**
 * @brief Poll RGB service and consume all currently available UART0 abort bytes.
 *
 * @param context Active per-test context; null forces safe shutdown.
 * @return true when abort is requested or context is null; otherwise false.
 *
 * @note This function does not wait for UART input. Recognized abort immediately
 *       invokes safety_all_off().
 */
bool test_runner_poll_abort(test_context_t *context);

/**
 * @brief Feed one console byte to the exact lowercase "abort" recognizer.
 *
 * @param context Per-execution parser state; null is treated as aborted.
 * @param byte    Next raw console byte.
 * @return true once "abort" has matched or an abort was already requested.
 *
 * @note Matching state belongs to one context and tolerates a new 'a' as the
 *       start of a new candidate after a mismatch.
 */
bool test_runner_consume_abort_byte(test_context_t *context, uint8_t byte);

#endif
