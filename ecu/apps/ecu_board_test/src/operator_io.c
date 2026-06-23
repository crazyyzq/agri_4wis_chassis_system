#include <stdio.h>
#include <stdlib.h>
#include "operator_io.h"

bool operator_read_line(char *buffer, size_t capacity)
{
    if (buffer == NULL || capacity < 2U) return false;
    size_t length = 0U;
    for (;;) {
        int c = getchar();
        if (c == EOF) continue;
        if (c == '\r' || c == '\n') {
            if (length == 0U) continue;
            putchar('\n');
            buffer[length] = '\0';
            return true;
        }
        if ((c == '\b' || c == 0x7F) && length > 0U) {
            --length;
            printf("\b \b");
        } else if (c >= 0x20 && c <= 0x7E && length + 1U < capacity) {
            buffer[length++] = (char)c;
            putchar(c);
        }
    }
}

bool operator_confirm(const char *prompt)
{
    char line[8];
    printf("%s [y/n]: ", prompt);
    if (!operator_read_line(line, sizeof(line))) return false;
    return line[0] == 'y' || line[0] == 'Y';
}

bool operator_read_u32(const char *prompt, uint32_t *value)
{
    char line[24];
    char *end;
    printf("%s: ", prompt);
    if (!operator_read_line(line, sizeof(line))) return false;
    unsigned long parsed = strtoul(line, &end, 10);
    if (*line == '\0' || *end != '\0' || parsed > UINT32_MAX) return false;
    *value = (uint32_t)parsed;
    return true;
}
