/* Shared result, requirement and aggregate board-session data model. */
#ifndef ECU_TEST_TYPES_H
#define ECU_TEST_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { TEST_PASS, TEST_FAIL, TEST_SKIP, TEST_BLOCKED } test_status_t;
typedef enum { TEST_REQUIRED, TEST_OPTIONAL } test_requirement_t;
typedef enum {
    TEST_BOARD_PASS,
    TEST_BOARD_FAIL,
    TEST_BOARD_INCOMPLETE,
    TEST_BOARD_ABORTED
} test_board_status_t;

typedef struct {
    char board_serial[24];
    uint16_t pass_count;
    uint16_t fail_count;
    uint16_t skip_count;
    uint16_t blocked_count;
    bool aborted;
    /* Required-only counters prevent an optional skip from blocking release. */
    uint16_t required_fail_count;
    uint16_t required_blocked_count;
    uint16_t required_count;
} test_session_t;

/* Reset all counters and copy a serial into the fixed 23-character field. */
void test_session_init(test_session_t *session, const char *serial);
void test_session_add(test_session_t *session, test_requirement_t requirement,
                      test_status_t status);
test_board_status_t test_session_status(const test_session_t *session);
const char *test_status_name(test_status_t status);
const char *test_board_status_name(test_board_status_t status);

#endif
