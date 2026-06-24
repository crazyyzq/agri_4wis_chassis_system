/* UART0 operator input with local echo and cooperative RGB servicing. */
#ifndef ECU_OPERATOR_IO_H
#define ECU_OPERATOR_IO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* Block until a nonempty printable line; capacity includes the terminator. */
bool operator_read_line(char *buffer, size_t capacity);
/* Prompt for y/Y; all other complete lines mean false. */
bool operator_confirm(const char *prompt);
/* Read a base-10 value in the inclusive uint32_t range. */
bool operator_read_u32(const char *prompt, uint32_t *value);
#endif
