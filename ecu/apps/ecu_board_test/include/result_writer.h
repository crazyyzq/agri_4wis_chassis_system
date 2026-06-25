/* Human-readable and bounded JSON output for one test result. */
#ifndef ECU_RESULT_WRITER_H
#define ECU_RESULT_WRITER_H
#include <stddef.h>
#include <stdint.h>
#include "test_types.h"
#define RESULT_JSON_MAX 384U
typedef struct {
    const char *id;
    test_requirement_t requirement;
    test_status_t status;
    uint16_t error_code;
    const char *detail;
} test_result_t;
/**
 * @brief Serialize one test result as a bounded single-line JSON object.
 *
 * @param result       Test result to serialize.
 * @param board_serial Board serial text; null is encoded as an empty string.
 * @param output       Caller-owned destination buffer.
 * @param capacity     Total bytes available in output, including the NUL byte.
 * @return Zero on success, or error code 0x0104 for null required arguments or
 *         insufficient output capacity.
 *
 * @note Quotes and backslashes are escaped, control characters are replaced by
 *       spaces, and successful output is NUL-terminated without a newline.
 */
int result_writer_json(const test_result_t *result, const char *board_serial,
                       char *output, size_t capacity);
/**
 * @brief Print one human-readable result line followed by its JSON line.
 *
 * @param result       Test result to print; null prints a writer-failure line.
 * @param board_serial Board serial passed to result_writer_json(); may be null.
 *
 * @note Each successful format is emitted as a complete newline-terminated
 *       console line. JSON failures emit a separate RESULT_WRITER failure line.
 */
void result_writer_print(const test_result_t *result, const char *board_serial);
#endif
