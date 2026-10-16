#include "fw_image.h"
#include "crc.h"
#include "err.h"
#include <string.h>

#define FW_CHUNK  64U

static uint32_t header_crc(const fw_header_t *h)
{
    return crc32((const uint8_t *)h, offsetof(fw_header_t, hdr_crc));
}

void fw_header_make(fw_header_t *h, uint32_t version, uint32_t size, uint32_t crc)
{
    h->magic   = FW_MAGIC;
    h->version = version;
    h->size    = size;
    h->crc     = crc;
    h->hdr_crc = header_crc(h);
}

int fw_header_check(const fw_header_t *h, uint32_t max_size)
{
    if (h->magic != FW_MAGIC) {
        return ERR_NOT_FOUND;
    }
    if (h->hdr_crc != header_crc(h)) {
        return ERR_CRC;
    }
    if (h->size == 0U || h->size > max_size) {
        return ERR_PARAM;
    }
    return ERR_OK;
}

int fw_slot_verify(const flash_dev_t *f, uint32_t slot, uint32_t max_size, fw_header_t *out)
{
    fw_header_t h;
    int rc = f->read(f->ctx, slot, &h, sizeof h);
    if (rc == ERR_OK) {
        rc = fw_header_check(&h, max_size);
    }
    if (rc != ERR_OK) {
        return rc;
    }
    uint8_t buf[FW_CHUNK];
    uint32_t crc = 0U;
    for (uint32_t off = 0U; off < h.size; off += FW_CHUNK) {
        uint32_t n = (h.size - off < FW_CHUNK) ? h.size - off : FW_CHUNK;
        rc = f->read(f->ctx, slot + FW_HDR_SPACE + off, buf, n);
        if (rc != ERR_OK) {
            return rc;
        }
        crc = crc32_update(crc, buf, n);
    }
    if (crc != h.crc) {
        return ERR_CRC;
    }
    if (out != NULL) {
        *out = h;
    }
    return ERR_OK;
}

int fw_writer_begin(fw_writer_t *w, const flash_dev_t *f, uint32_t slot, uint32_t max_size)
{
    if (f == NULL || (slot % f->sector_size) != 0U || max_size == 0U) {
        return ERR_PARAM;
    }
    w->flash     = f;
    w->slot      = slot;
    w->max_size  = max_size;
    w->written   = 0U;
    w->crc       = 0U;
    w->erased_to = slot + f->sector_size;
    return f->erase_sector(f->ctx, slot);          /* header gone: slot is empty now */
}

int fw_writer_put(fw_writer_t *w, const void *data, size_t len)
{
    if (len == 0U) {
        return ERR_OK;
    }
    if ((uint64_t)w->written + len > w->max_size) {
        return ERR_FULL;
    }
    uint32_t addr = w->slot + FW_HDR_SPACE + w->written;
    while (addr + len > w->erased_to) {            /* erase sectors just before use */
        int rc = w->flash->erase_sector(w->flash->ctx, w->erased_to);
        if (rc != ERR_OK) {
            return rc;
        }
        w->erased_to += w->flash->sector_size;
    }
    int rc = w->flash->write(w->flash->ctx, addr, data, len);
    if (rc != ERR_OK) {
        return rc;
    }
    w->crc = crc32_update(w->crc, (const uint8_t *)data, len);
    w->written += (uint32_t)len;
    return ERR_OK;
}

int fw_writer_finish(fw_writer_t *w, uint32_t version, uint32_t expect_crc)
{
    if (w->written == 0U) {
        return ERR_EMPTY;
    }
    if (w->crc != expect_crc) {
        return ERR_CRC;                            /* header not written: slot stays empty */
    }
    fw_header_t h;
    fw_header_make(&h, version, w->written, w->crc);
    int rc = w->flash->write(w->flash->ctx, w->slot, &h, sizeof h);
    if (rc != ERR_OK) {
        return rc;
    }
    /* read the whole slot back: catches a bad program or a bad wire */
    return fw_slot_verify(w->flash, w->slot, w->max_size, NULL);
}

/*
 * The caller saves the state only AFTER the action succeeded. A power cut
 * during an install therefore leaves the state PENDING and the install is
 * simply done again at the next reset (the NEW slot is never modified).
 */
fw_action_t fw_boot_decide(fw_boot_t *st, bool new_ok, bool backup_ok, bool app_ok, uint32_t max_tries)
{
    switch ((fw_state_t)st->state) {
    case FW_STATE_PENDING:
        if (new_ok) {
            st->state        = FW_STATE_TRIAL;
            st->tries        = 0U;
            st->prev_version = app_ok ? st->version : 0U;
            return FW_ACT_INSTALL;                 /* caller sets st->version from the header */
        }
        st->state = FW_STATE_IDLE;                 /* NEW slot damaged: forget the request */
        break;
    case FW_STATE_TRIAL:
        st->tries++;
        if (st->tries > max_tries) {
            st->state = FW_STATE_IDLE;
            st->tries = 0U;
            if (backup_ok) {
                st->version = st->prev_version;
                return FW_ACT_ROLLBACK;            /* never confirmed: go back */
            }
            break;                                 /* nothing to go back to: keep it */
        }
        break;
    case FW_STATE_IDLE:
        break;
    default:                                       /* unknown value: treat as IDLE */
        st->state = FW_STATE_IDLE;
        st->tries = 0U;
        break;
    }
    if (!app_ok && backup_ok) {
        st->version = st->prev_version;
        return FW_ACT_ROLLBACK;                    /* internal flash damaged */
    }
    return FW_ACT_RUN;
}

bool fw_boot_confirm(fw_boot_t *st)
{
    if (st->state != FW_STATE_TRIAL) {
        return false;
    }
    st->state = FW_STATE_IDLE;
    st->tries = 0U;
    return true;
}

bool fw_vectors_ok(uint32_t sp, uint32_t reset, uint32_t app_start, uint32_t app_end,
                   uint32_t ram_start, uint32_t ram_end)
{
    bool sp_ok    = (sp > ram_start) && (sp <= ram_end) && ((sp & 3U) == 0U);
    bool reset_ok = ((reset & 1U) == 1U) && (reset > app_start) && (reset < app_end);   /* Thumb bit */
    return sp_ok && reset_ok;
}
