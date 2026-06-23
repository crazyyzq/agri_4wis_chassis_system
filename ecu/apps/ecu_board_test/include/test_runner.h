#ifndef ECU_TEST_RUNNER_H
#define ECU_TEST_RUNNER_H

#include <stdbool.h>
#include <stdint.h>
#include "test_types.h"

typedef struct {
    volatile bool abort_requested;
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

test_status_t test_runner_execute(const test_descriptor_t *descriptor,
                                  test_context_t *context);
bool test_runner_poll_abort(test_context_t *context);

#endif
