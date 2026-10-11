/**
 * @file    console.h
 * @brief   Command line on the ST-LINK virtual COM port (115200 8N1).
 *          Type "help" in any terminal (PuTTY, Tera Term, minicom).
 */
#ifndef CONSOLE_H
#define CONSOLE_H

void console_init(void);
void console_task(void *arg);

#endif /* CONSOLE_H */
