#include "cli.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

void cli_init(cli_t *cli, const cli_cmd_t *cmds, size_t ncmds, void (*write)(const char *s), bool echo)
{
    cli->cmds   = cmds;
    cli->ncmds  = ncmds;
    cli->write  = write;
    cli->prompt = "fan> ";
    cli->len    = 0U;
    cli->echo   = echo;
}

void cli_printf(cli_t *cli, const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    cli->write(buf);
}

int cli_tokenize(char *line, char *argv[], int max_args)
{
    int argc = 0;
    char *p = line;
    while (*p != '\0' && argc < max_args) {
        while (*p == ' ' || *p == '\t') {
            *p++ = '\0';
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            p++;
        }
    }
    return argc;
}

int cli_execute(cli_t *cli, char *line)
{
    char *argv[CLI_ARGS_MAX];
    int argc = cli_tokenize(line, argv, (int)CLI_ARGS_MAX);
    if (argc == 0) {
        return 0;
    }
    if (strcmp(argv[0], "help") == 0) {
        for (size_t i = 0U; i < cli->ncmds; i++) {
            cli_printf(cli, "  %-10s %s\r\n", cli->cmds[i].name, cli->cmds[i].help);
        }
        return 0;
    }
    for (size_t i = 0U; i < cli->ncmds; i++) {
        if (strcmp(argv[0], cli->cmds[i].name) == 0) {
            return cli->cmds[i].handler(cli, argc, argv);
        }
    }
    cli_printf(cli, "unknown command '%s', type help\r\n", argv[0]);
    return -1;
}

void cli_input(cli_t *cli, char c)
{
    if (c == '\r' || c == '\n') {
        if (cli->echo) {
            cli->write("\r\n");
        }
        cli->line[cli->len] = '\0';
        if (cli->len > 0U) {
            (void)cli_execute(cli, cli->line);
        }
        cli->len = 0U;
        cli->write(cli->prompt);
    } else if (c == '\b' || c == 0x7F) {           /* backspace / delete */
        if (cli->len > 0U) {
            cli->len--;
            if (cli->echo) {
                cli->write("\b \b");
            }
        }
    } else if (c >= ' ' && cli->len < CLI_LINE_MAX - 1U) {
        cli->line[cli->len++] = c;
        if (cli->echo) {
            char s[2] = { c, '\0' };
            cli->write(s);
        }
    }
}
