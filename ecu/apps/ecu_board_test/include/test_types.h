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
    /* Zero means this is an ad-hoc/partial session, never a board PASS. */
    uint16_t expected_required_count;
} test_session_t;

/**
 * @brief Reset a test session and assign its board serial number.
 *
 * @param session Caller-owned session to initialize; null is ignored.
 * @param serial  NUL-terminated serial to copy; null leaves an empty serial.
 *
 * @note At most 23 serial characters are retained and the field is always
 *       NUL-terminated.
 */
void test_session_init(test_session_t *session, const char *serial);
/**
 * @brief Set the number of required cases expected in a complete board run.
 *
 * @param session Caller-owned session to update; null is ignored.
 * @param count   Required-case total; zero explicitly denotes a partial or
 *                ad-hoc session that cannot report TEST_BOARD_PASS.
 */
void test_session_set_expected_required(test_session_t *session, uint16_t count);
/**
 * @brief Accumulate one test outcome into a board session.
 *
 * @param session     Caller-owned session to update; null is ignored.
 * @param requirement Whether the case is required or optional.
 * @param status      PASS, FAIL, SKIP, or BLOCKED result to count; invalid
 *                    values are ignored.
 *
 * @note A required SKIP is counted as required-blocked because it prevents a
 *       complete release result. Optional outcomes never affect release status.
 */
void test_session_add(test_session_t *session, test_requirement_t requirement,
                      test_status_t status);
/**
 * @brief Derive the release-oriented aggregate status of a board session.
 *
 * @param session Session to evaluate.
 * @return TEST_BOARD_ABORTED when marked aborted; otherwise TEST_BOARD_FAIL on
 *         a required failure, TEST_BOARD_INCOMPLETE for a null/partial/mismatched
 *         or required-blocked run, and TEST_BOARD_PASS only when the nonzero
 *         expected required count is matched exactly with no blocking outcome.
 */
test_board_status_t test_session_status(const test_session_t *session);
/**
 * @brief Return the printable name of an individual test status.
 *
 * @param status Status value to translate.
 * @return A pointer to immutable static text, or "INVALID" for an unknown value.
 */
const char *test_status_name(test_status_t status);
/**
 * @brief Return the printable name of an aggregate board status.
 *
 * @param status Board status value to translate.
 * @return A pointer to immutable static text, or "INVALID" for an unknown value.
 */
const char *test_board_status_name(test_board_status_t status);

#endif
