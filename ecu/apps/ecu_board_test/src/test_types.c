/* Aggregate individual outcomes into a release-oriented board session status. */
#include <string.h>
#include "test_types.h"

void test_session_init(test_session_t *session, const char *serial)
{
    if (session == NULL) return;
    memset(session, 0, sizeof(*session));
    if (serial != NULL) {
        strncpy(session->board_serial, serial, sizeof(session->board_serial) - 1U);
        session->board_serial[sizeof(session->board_serial) - 1U] = '\0';
    }
}

void test_session_set_expected_required(test_session_t *session, uint16_t count)
{
    if (session == NULL) return;
    session->expected_required_count = count;
}

void test_session_add(test_session_t *session, test_requirement_t requirement,
                      test_status_t status)
{
    if (session == NULL) return;
    switch (status) {
    case TEST_PASS: ++session->pass_count; break;
    case TEST_FAIL: ++session->fail_count; break;
    case TEST_SKIP: ++session->skip_count; break;
    case TEST_BLOCKED: ++session->blocked_count; break;
    default: return;
    }
    if (requirement == TEST_REQUIRED) {
        ++session->required_count;
        if (status == TEST_FAIL) ++session->required_fail_count;
        if (status == TEST_BLOCKED || status == TEST_SKIP) ++session->required_blocked_count;
    }
}

test_board_status_t test_session_status(const test_session_t *session)
{
    /* Status precedence is ABORTED, required FAIL, INCOMPLETE, then PASS. */
    if (session == NULL) return TEST_BOARD_INCOMPLETE;
    if (session->aborted) return TEST_BOARD_ABORTED;
    if (session->required_fail_count != 0U) return TEST_BOARD_FAIL;
    if (session->expected_required_count == 0U ||
        session->required_count != session->expected_required_count) {
        return TEST_BOARD_INCOMPLETE;
    }
    if (session->required_blocked_count != 0U) return TEST_BOARD_INCOMPLETE;
    return TEST_BOARD_PASS;
}

const char *test_status_name(test_status_t status)
{
    static const char *names[] = { "PASS", "FAIL", "SKIP", "BLOCKED" };
    return (uint32_t)status < 4U ? names[status] : "INVALID";
}

const char *test_board_status_name(test_board_status_t status)
{
    static const char *names[] = { "PASS", "FAIL", "INCOMPLETE", "ABORTED" };
    return (uint32_t)status < 4U ? names[status] : "INVALID";
}
