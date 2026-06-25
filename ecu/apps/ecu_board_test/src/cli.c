/* Bounded, allocation-free parser for the ECU test console. */
#include <ctype.h>
#include <string.h>
#include "cli.h"

/**
 * @brief Advance over console separator characters without modifying input.
 *
 * @param p Current position in a valid NUL-terminated command string.
 * @return Pointer to the first character that is neither a space nor a tab.
 */
static const char *skip_space(const char *p) { while (*p == ' ' || *p == '\t') ++p; return p; }

cli_action_t cli_parse(const char *line, cli_command_t *command)
{
    if (line == NULL || command == NULL) return CLI_INVALID;
    memset(command, 0, sizeof(*command));
    line = skip_space(line);
    size_t length = strnlen(line, CLI_LINE_MAX + 1U);
    if (length == 0U || length > CLI_LINE_MAX) return CLI_INVALID;
    while (length > 0U && isspace((unsigned char)line[length - 1U])) --length;
    if (length == 4U && strncmp(line, "list", 4U) == 0) return command->action = CLI_LIST;
    if (length == 4U && strncmp(line, "help", 4U) == 0) return command->action = CLI_HELP;
    if (length == 6U && strncmp(line, "status", 6U) == 0) return command->action = CLI_STATUS;
    if (length == 6U && strncmp(line, "report", 6U) == 0) return command->action = CLI_REPORT;
    if (length == 5U && strncmp(line, "abort", 5U) == 0) return command->action = CLI_ABORT;
    if (length == 12U && strncmp(line, "SELFTEST.ALL", 12U) == 0) return command->action = CLI_SELFTEST;

    const char *arg = NULL;
    if (length > 4U && strncmp(line, "run ", 4U) == 0) arg = skip_space(line + 4U);
    if (arg != NULL && *arg != '\0') {
        size_t arg_len = length - (size_t)(arg - line);
        if (arg_len == 3U && strncmp(arg, "all", 3U) == 0) return command->action = CLI_RUN_ALL;
        memcpy(command->argument, arg, arg_len);
        command->argument[arg_len] = '\0';
        return command->action = CLI_RUN_ONE;
    }
    if (length > 6U && strncmp(line, "board ", 6U) == 0) {
        arg = skip_space(line + 6U);
        size_t arg_len = length - (size_t)(arg - line);
        if (arg_len > 0U && arg_len < sizeof(command->argument)) {
            memcpy(command->argument, arg, arg_len);
            command->argument[arg_len] = '\0';
            return command->action = CLI_BOARD;
        }
    }
    return CLI_INVALID;
}
