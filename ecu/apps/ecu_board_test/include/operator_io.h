/* UART0 operator input with local echo and cooperative RGB servicing. */
#ifndef ECU_OPERATOR_IO_H
#define ECU_OPERATOR_IO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/**
 * @brief Read and locally echo one nonempty printable UART0 input line.
 *
 * @param buffer   Caller-owned destination for the NUL-terminated line.
 * @param capacity Total destination capacity, including the terminator; at
 *                 least two bytes are required.
 * @return true after a nonempty line is received, or false for invalid args.
 *
 * @note The call blocks indefinitely, ignores empty CR/LF lines, accepts ASCII
 *       0x20..0x7E, supports backspace, and calls status_led_poll() while waiting.
 */
bool operator_read_line(char *buffer, size_t capacity);
/**
 * @brief Ask the operator for a yes/no confirmation on UART0.
 *
 * @param prompt NUL-terminated prompt text; null is rejected.
 * @return true only when the first response character is 'y' or 'Y'; false for
 *         invalid input or every other complete response.
 *
 * @note Blocks through operator_read_line().
 */
bool operator_confirm(const char *prompt);
/**
 * @brief Read one complete unsigned decimal value from UART0.
 *
 * @param prompt NUL-terminated prompt text; null is rejected.
 * @param value  Caller-owned destination written only after valid conversion.
 * @return true for a complete base-10 value in the inclusive uint32_t range;
 *         false for invalid arguments, trailing characters, or overflow.
 *
 * @note Blocks through operator_read_line().
 */
bool operator_read_u32(const char *prompt, uint32_t *value);
#endif
