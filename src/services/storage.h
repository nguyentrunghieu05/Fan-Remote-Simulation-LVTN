/**
 * @file    storage.h
 * @brief   Everything that lives on the external SPI flash (W25Q64).
 *
 *   0x000000  sector 0   settings, copy A   \  config_store (A/B, CRC-32)
 *   0x001000  sector 1   settings, copy B   /
 *   0x002000  sectors 2, 3  update state A/B -> config_store (fw_boot_t)
 *   0x010000  ...        measurement log    -> datalog (ring, CRC-16), up to 7 MB
 *   0x700000  512 KB     firmware slot NEW  -> fw_image (installed by the bootloader)
 *   0x780000  512 KB     firmware slot BACKUP (previous image, for rollback)
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
#include "fw_image.h"

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

/* Firmware update: receive an image into the NEW slot, then ask the
 * bootloader to install it at the next reset (see boot/boot.c). */
int  storage_fw_begin(void);
int  storage_fw_put(const void *data, size_t len);
int  storage_fw_finish(uint32_t version, uint32_t crc);
int  storage_fw_slot(uint32_t slot, fw_header_t *h);
int  storage_fw_state(fw_boot_t *st);            /* IDLE with version 0 if never saved */
int  storage_fw_request_install(void);           /* state PENDING, needs a valid NEW slot */
/** TRIAL -> IDLE: the running image works. @return ERR_OK, or ERR_STATE if nothing to confirm. */
int  storage_fw_confirm(void);

/** Task: appends one sample every cfg.log_period_s and confirms a new
 *  firmware once it has run for FW_CONFIRM_S seconds. */
void storage_task(void *arg);

#endif /* STORAGE_H */
