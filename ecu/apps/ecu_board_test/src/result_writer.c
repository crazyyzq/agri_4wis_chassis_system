/* Emit each test result in operator text and escaped single-line JSON. */
#include <stdio.h>
#include <string.h>
#include "result_writer.h"

static bool append_escaped(char *output, size_t capacity, size_t *used, const char *input)
{
    if (input == NULL) input = "";
    while (*input != '\0') {
        char c = *input++;
        if (c == '"' || c == '\\') {
            if (*used + 2U >= capacity) return false;
            output[(*used)++] = '\\';
        } else if ((unsigned char)c < 0x20U) {
            c = ' ';
        }
        if (*used + 1U >= capacity) return false;
        output[(*used)++] = c;
    }
    output[*used] = '\0';
    return true;
}

int result_writer_json(const test_result_t *result, const char *board_serial,
                       char *output, size_t capacity)
{
    if (result == NULL || output == NULL || capacity == 0U) return 0x0104;
    int count = snprintf(output, capacity, "{\"board\":\"");
    if (count < 0 || (size_t)count >= capacity) return 0x0104;
    size_t used = (size_t)count;
    if (!append_escaped(output, capacity, &used, board_serial)) return 0x0104;
    count = snprintf(output + used, capacity - used, "\",\"id\":\"");
    if (count < 0 || (size_t)count >= capacity - used) return 0x0104;
    used += (size_t)count;
    if (!append_escaped(output, capacity, &used, result->id)) return 0x0104;
    count = snprintf(output + used, capacity - used,
        "\",\"requirement\":\"%s\",\"status\":\"%s\",\"error\":%u,\"detail\":\"",
        result->requirement == TEST_REQUIRED ? "required" : "optional",
        test_status_name(result->status), result->error_code);
    if (count < 0 || (size_t)count >= capacity - used) return 0x0104;
    used += (size_t)count;
    if (!append_escaped(output, capacity, &used, result->detail)) return 0x0104;
    if (used + 3U > capacity) return 0x0104;
    output[used++] = '"'; output[used++] = '}'; output[used] = '\0';
    return 0;
}

void result_writer_print(const test_result_t *result, const char *board_serial)
{
    if (result == NULL) {
        printf("RESULT_WRITER FAIL error=0x0104\n");
        return;
    }
    char json[RESULT_JSON_MAX];
    printf("RESULT %-22s %s error=0x%04x %s\n", result->id,
           test_status_name(result->status), result->error_code,
           result->detail != NULL ? result->detail : "");
    if (result_writer_json(result, board_serial, json, sizeof(json)) == 0) printf("%s\n", json);
    else printf("RESULT_WRITER FAIL error=0x0104\n");
}
