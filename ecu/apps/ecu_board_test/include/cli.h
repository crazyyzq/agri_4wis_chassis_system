/* Side-effect-free parser for the bounded UART operator command language. */
#ifndef ECU_CLI_H
#define ECU_CLI_H

#include <stddef.h>

#define CLI_LINE_MAX 96U
typedef enum {
    CLI_INVALID, CLI_HELP, CLI_LIST, CLI_RUN_ONE, CLI_RUN_ALL,
    CLI_BOARD, CLI_STATUS, CLI_REPORT, CLI_ABORT, CLI_SELFTEST
} cli_action_t;

typedef struct {
    cli_action_t action;
    char argument[CLI_LINE_MAX];
} cli_command_t;

/**
 * @brief Parse one bounded operator-console command without side effects.
 *
 * @param line    NUL-terminated input containing at most CLI_LINE_MAX bytes,
 *                excluding the terminator.
 * @param command Caller-owned destination cleared before any non-null input is
 *                parsed; receives the action and optional argument.
 * @return The parsed action, or CLI_INVALID for null arguments, an empty line,
 *         an overlength line, or unrecognized syntax.
 *
 * @note Leading spaces and tabs and trailing whitespace are ignored. The
 *       returned argument is always NUL-terminated when parsing succeeds.
 */
cli_action_t cli_parse(const char *line, cli_command_t *command);

#endif
