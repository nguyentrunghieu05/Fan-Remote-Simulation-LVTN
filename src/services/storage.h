/**
 * @file    storage.h
 * @brief   Everything that lives on the external SPI flash (W25Q64).
 *
 *   0x000000  sector 0   settings, copy A   \  config_store (A/B, CRC-32)
 *   0x001000  sector 1   settings, copy B   /
 *   0x010000  ...        measurement log    -> datalog (ring, CRC-16)
 *
 * All functions are thread safe (one mutex) and return ERR_STATE when no
 * flash chip was found at boot, so the controller still works without it.
 */
#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "datalog.h"
#include "fan_logic.h"

int  storage_init(void);                   /* call before the scheduler starts */
bool storage_ok(void);
uint32_t storage_jedec_id(void);
uint32_t storage_size(void);

int  storage_load_config(fan_config_t *cfg);
int  storage_save_config(const fan_config_t *cfg);

int  storage_log_append(const datalog_sample_t *s);
uint32_t storage_log_count(void);
uint32_t storage_log_capacity(void);
int  storage_log_read(uint32_t index, datalog_sample_t *s, uint32_t *seq);
int  storage_log_next_unsent(uint32_t *slot, uint32_t *seq, datalog_sample_t *s);
int  storage_log_mark_sent(uint32_t slot);
int  storage_log_format(void);

/** Task: appends one sample every cfg.log_period_s. */
void storage_task(void *arg);

#endif /* STORAGE_H */
