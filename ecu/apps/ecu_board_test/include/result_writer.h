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
/* Return zero on success or 0x0104 when arguments/capacity are invalid. */
int result_writer_json(const test_result_t *result, const char *board_serial,
                       char *output, size_t capacity);
/* Print the text result and a second machine-readable JSON line. */
void result_writer_print(const test_result_t *result, const char *board_serial);
#endif
