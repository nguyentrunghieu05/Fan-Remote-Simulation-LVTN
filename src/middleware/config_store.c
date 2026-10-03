#include "config_store.h"
#include "crc.h"
#include "err.h"
#include <stdbool.h>
#include <string.h>

#define CFG_MAGIC  0x46414E43UL   /* "FANC" */
#define CFG_CHUNK  64U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint16_t len;
    uint16_t reserved;
    uint32_t crc;           /* CRC-32 of the payload */
} cfg_header_t;

void config_store_init(config_store_t *cs, const flash_dev_t *flash, uint32_t sector_a, uint32_t sector_b)
{
    cs->flash   = flash;
    cs->addr[0] = sector_a;
    cs->addr[1] = sector_b;
    cs->seq     = 0U;
    cs->current = -1;
}

/* CRC of the payload stored after the header, read in small chunks so no
 * big buffer is needed. */
static int payload_crc(const config_store_t *cs, uint32_t addr, size_t len, uint32_t *crc)
{
    uint8_t buf[CFG_CHUNK];
    uint32_t c = 0U;
    for (size_t off = 0U; off < len; off += CFG_CHUNK) {
        size_t n = (len - off < CFG_CHUNK) ? len - off : CFG_CHUNK;
        int rc = cs->flash->read(cs->flash->ctx, addr + sizeof(cfg_header_t) + off, buf, n);
        if (rc != ERR_OK) {
            return rc;
        }
        c = crc32_update(c, buf, n);
    }
    *crc = c;
    return ERR_OK;
}

static int check_copy(const config_store_t *cs, int i, size_t len, cfg_header_t *h)
{
    int rc = cs->flash->read(cs->flash->ctx, cs->addr[i], h, sizeof(*h));
    if (rc != ERR_OK) {
        return rc;
    }
    if (h->magic != CFG_MAGIC || h->len != len) {
        return ERR_NOT_FOUND;               /* empty, or saved by another firmware layout */
    }
    uint32_t crc;
    rc = payload_crc(cs, cs->addr[i], len, &crc);
    if (rc != ERR_OK) {
        return rc;
    }
    return (crc == h->crc) ? ERR_OK : ERR_CRC;
}

int config_store_load(config_store_t *cs, void *data, size_t len)
{
    cfg_header_t h[2];
    bool ok[2];
    for (int i = 0; i < 2; i++) {
        ok[i] = (check_copy(cs, i, len, &h[i]) == ERR_OK);
    }
    int pick;
    if (ok[0] && ok[1]) {
        pick = (h[1].seq > h[0].seq) ? 1 : 0;
    } else if (ok[0] || ok[1]) {
        pick = ok[0] ? 0 : 1;
    } else {
        cs->current = -1;
        cs->seq     = 0U;
        return ERR_NOT_FOUND;
    }
    cs->current = pick;
    cs->seq     = h[pick].seq;
    return cs->flash->read(cs->flash->ctx, cs->addr[pick] + sizeof(cfg_header_t), data, len);
}

int config_store_save(config_store_t *cs, const void *data, size_t len)
{
    if (len == 0U || len + sizeof(cfg_header_t) > cs->flash->sector_size) {
        return ERR_PARAM;
    }
    int target = (cs->current == 0) ? 1 : 0;
    uint32_t addr = cs->addr[target];

    cfg_header_t h = {
        .magic = CFG_MAGIC,
        .seq   = cs->seq + 1U,
        .len   = (uint16_t)len,
        .crc   = crc32((const uint8_t *)data, len),
    };
    int rc = cs->flash->erase_sector(cs->flash->ctx, addr);
    if (rc == ERR_OK) {
        /* payload first, header last: a copy without header is never valid */
        rc = cs->flash->write(cs->flash->ctx, addr + sizeof h, data, len);
    }
    if (rc == ERR_OK) {
        rc = cs->flash->write(cs->flash->ctx, addr, &h, sizeof h);
    }
    if (rc == ERR_OK) {
        cfg_header_t check;
        rc = check_copy(cs, target, len, &check);       /* verify read-back */
    }
    if (rc == ERR_OK) {
        cs->current = target;
        cs->seq     = h.seq;
    }
    return rc;
}
