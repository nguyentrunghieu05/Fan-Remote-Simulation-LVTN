/**
 * @file    supervisor.h
 * @brief   Software watchdog per task, feeding the hardware IWDG.
 *
 * Every task registers with its own deadline and calls sv_kick() each
 * loop. The supervisor task (lowest priority) refreshes the IWDG only if
 * EVERY task kicked in time. So the MCU resets if any task hangs (dead
 * lock, endless loop) or if the CPU is starved and even the lowest
 * priority cannot run. The log says which task missed its deadline.
 */
#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

#define SV_MAX_CLIENTS  8

int  sv_register(const char *name, uint32_t timeout_ms);
void sv_kick(int id);
/** Long blocking job (flash erase): keep the IWDG fed, ignore task deadlines. */
void sv_maintenance(bool on);
void sv_task(void *arg);

#endif /* SUPERVISOR_H */
