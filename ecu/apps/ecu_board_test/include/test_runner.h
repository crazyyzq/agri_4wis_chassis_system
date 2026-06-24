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

/* Execute one descriptor and always finish with safety_all_off(). */
test_status_t test_runner_execute(const test_descriptor_t *descriptor,
                                  test_context_t *context);
/* Service status LED and consume available UART0 bytes without blocking. */
bool test_runner_poll_abort(test_context_t *context);

/* Feed one console byte into the exact lowercase "abort" recognizer. */
bool test_runner_consume_abort_byte(test_context_t *context, uint8_t byte);

#endif
