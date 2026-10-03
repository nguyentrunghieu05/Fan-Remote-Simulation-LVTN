/* RAM model of a NOR flash: erase sets 0xFF, program can only clear bits.
 * Can inject a "power cut" after N programmed bytes. */
#ifndef FAKE_FLASH_H
#define FAKE_FLASH_H
#include "bus.h"
typedef struct {
    uint8_t *mem;
    uint32_t size, sector;
    long     cut_after;     /* -1 = never; else bytes left before power cut */
    unsigned erases, writes;
} fake_flash_t;
void fake_flash_init(fake_flash_t *f, flash_dev_t *dev, uint32_t size, uint32_t sector);
void fake_flash_free(fake_flash_t *f);
#endif
