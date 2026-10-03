/**
 * @file    log.h
 * @brief   Timestamped debug output on the console UART.
 */
#ifndef LOG_H
#define LOG_H

void log_printf(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void console_write(const char *s);

#define LOGI(tag, ...)  log_printf(tag, __VA_ARGS__)

#endif /* LOG_H */
