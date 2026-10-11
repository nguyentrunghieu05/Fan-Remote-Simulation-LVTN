#include "log.h"
#include "bsp.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

void console_write(const char *s)
{
    (void)bsp_uart_write(BSP_UART_CONSOLE, s, strlen(s));
}

void log_printf(const char *tag, const char *fmt, ...)
{
    char buf[160];
    uint32_t ms = bsp_millis();
    int n = snprintf(buf, sizeof buf, "[%6lu.%03lu] %-5s ", (unsigned long)(ms / 1000U),
                     (unsigned long)(ms % 1000U), tag);
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(buf + n, sizeof buf - (size_t)n, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof buf - 3) {
        n = (int)sizeof buf - 3;
    }
    buf[n++] = '\r';
    buf[n++] = '\n';
    buf[n]   = '\0';
    console_write(buf);
}
