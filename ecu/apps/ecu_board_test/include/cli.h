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

cli_action_t cli_parse(const char *line, cli_command_t *command);

#endif
