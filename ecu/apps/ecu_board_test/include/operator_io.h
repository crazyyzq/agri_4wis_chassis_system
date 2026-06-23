#ifndef ECU_OPERATOR_IO_H
#define ECU_OPERATOR_IO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
bool operator_read_line(char *buffer, size_t capacity);
bool operator_confirm(const char *prompt);
bool operator_read_u32(const char *prompt, uint32_t *value);
#endif
