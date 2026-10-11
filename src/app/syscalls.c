/**
 * @file    syscalls.c
 * @brief   Minimal newlib system calls. printf() goes to the console UART;
 *          the rest exist only so the C library links.
 */
#include <errno.h>
#include <sys/stat.h>
#include "bsp.h"

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    return (int)bsp_uart_write(BSP_UART_CONSOLE, buf, (size_t)len);
}

int _read(int fd, char *buf, int len)    { (void)fd; (void)buf; (void)len; errno = ENOSYS; return -1; }
int _close(int fd)                       { (void)fd; return -1; }
int _lseek(int fd, int off, int dir)     { (void)fd; (void)off; (void)dir; return 0; }
int _isatty(int fd)                      { (void)fd; return 1; }
int _fstat(int fd, struct stat *st)      { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _getpid(void)                        { return 1; }
int _kill(int pid, int sig)              { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status)                   { (void)status; for (;;) { } }
