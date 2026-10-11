/**
 * @file    cli.h
 * @brief   Tiny command line interpreter for the UART console.
 *
 * Characters are fed one at a time (from the UART RX task). On Enter the
 * line is split into argc/argv and matched against a command table. Output
 * goes through a print callback so the module is testable on a PC.
 */
#ifndef CLI_H
#define CLI_H

#include <stddef.h>
#include <stdbool.h>

#define CLI_LINE_MAX   96U
#define CLI_ARGS_MAX   8U

typedef struct cli cli_t;

typedef int (*cli_handler_t)(cli_t *cli, int argc, char *argv[]);

typedef struct {
    const char   *name;
    const char   *help;
    cli_handler_t handler;
} cli_cmd_t;

struct cli {
    const cli_cmd_t *cmds;
    size_t           ncmds;
    void           (*write)(const char *s);   /* output sink */
    const char      *prompt;
    char             line[CLI_LINE_MAX];
    size_t           len;
    bool             echo;
};

void cli_init(cli_t *cli, const cli_cmd_t *cmds, size_t ncmds, void (*write)(const char *s), bool echo);
/** Feed one received character. */
void cli_input(cli_t *cli, char c);
/** Execute a full line (used by cli_input and by tests). @return handler result, -1 if unknown. */
int  cli_execute(cli_t *cli, char *line);
void cli_printf(cli_t *cli, const char *fmt, ...);
/** Split a line in place. @return argc. */
int  cli_tokenize(char *line, char *argv[], int max_args);

#endif /* CLI_H */
