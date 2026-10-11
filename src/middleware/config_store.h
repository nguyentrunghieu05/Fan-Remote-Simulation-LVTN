/**
 * @file    config_store.h
 * @brief   Power-loss safe settings storage (A/B copies on flash).
 *
 * Two sectors hold two copies. A save always writes the copy that is NOT
 * the current one, with a higher sequence number and a CRC-32. If power is
 * lost during the save, the old copy is still intact and is loaded at the
 * next boot. The payload is opaque bytes: the application decides what a
 * configuration contains.
 */
#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdint.h>
#include <stddef.h>
#include "bus.h"

typedef struct {
    const flash_dev_t *flash;
    uint32_t addr[2];       /* sector A and sector B */
    uint32_t seq;           /* sequence of the copy currently valid */
    int      current;       /* 0 = A, 1 = B, -1 = none */
} config_store_t;

void config_store_init(config_store_t *cs, const flash_dev_t *flash, uint32_t sector_a, uint32_t sector_b);
/** @return ERR_OK, or ERR_NOT_FOUND when no valid copy exists (use defaults). */
int  config_store_load(config_store_t *cs, void *data, size_t len);
int  config_store_save(config_store_t *cs, const void *data, size_t len);

#endif /* CONFIG_STORE_H */
