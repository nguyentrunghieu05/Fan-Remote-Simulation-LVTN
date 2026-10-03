#include "fake_flash.h"
#include "err.h"
#include <stdlib.h>
#include <string.h>

static int ff_read(void *ctx, uint32_t a, void *b, size_t n)
{
    fake_flash_t *f = ctx;
    if (a + n > f->size) return ERR_PARAM;
    memcpy(b, f->mem + a, n);
    return ERR_OK;
}
static int ff_write(void *ctx, uint32_t a, const void *b, size_t n)
{
    fake_flash_t *f = ctx;
    const uint8_t *p = b;
    if (a + n > f->size) return ERR_PARAM;
    f->writes++;
    for (size_t i = 0; i < n; i++) {
        if (f->cut_after == 0) return ERR_IO;          /* power gone */
        if (f->cut_after > 0) f->cut_after--;
        f->mem[a + i] &= p[i];                          /* NOR: 1 -> 0 only */
    }
    return ERR_OK;
}
static int ff_erase(void *ctx, uint32_t a)
{
    fake_flash_t *f = ctx;
    a -= a % f->sector;
    if (a >= f->size) return ERR_PARAM;
    f->erases++;
    memset(f->mem + a, 0xFF, f->sector);
    return ERR_OK;
}
void fake_flash_init(fake_flash_t *f, flash_dev_t *dev, uint32_t size, uint32_t sector)
{
    f->mem = malloc(size);
    memset(f->mem, 0xFF, size);
    f->size = size; f->sector = sector; f->cut_after = -1; f->erases = f->writes = 0;
    dev->read = ff_read; dev->write = ff_write; dev->erase_sector = ff_erase;
    dev->size = size; dev->sector_size = sector; dev->ctx = f;
}
void fake_flash_free(fake_flash_t *f) { free(f->mem); }
