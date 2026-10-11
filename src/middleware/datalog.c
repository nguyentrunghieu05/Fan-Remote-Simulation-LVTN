#include "datalog.h"
#include "crc.h"
#include "err.h"
#include <string.h>

#define REC_MAGIC     0xA55AU
#define REC_VERSION   1U
#define FLAG_UNSENT   0xFFU
#define FLAG_SENT     0x00U
#define FLAGS_OFFSET  2U

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t  flags;
    uint8_t  version;
    uint32_t seq;
    uint32_t timestamp;
    int16_t  temp_c100;
    uint16_t hum_c100;
    uint16_t rpm;
    uint16_t duty_c10;
    uint8_t  state;
    uint8_t  faults;
    uint8_t  reserved[8];
    uint16_t crc;
} record_t;

_Static_assert(sizeof(record_t) == DATALOG_RECORD_SIZE, "record must be 32 bytes");

static uint32_t slot_addr(const datalog_t *log, uint32_t slot)
{
    return log->base + slot * DATALOG_RECORD_SIZE;
}

/* CRC covers the whole record except the CRC itself, with flags forced to
 * 0xFF so that marking a record as sent does not invalidate it. */
static uint16_t record_crc(const record_t *r)
{
    record_t tmp = *r;
    tmp.flags = FLAG_UNSENT;
    return crc16_modbus((const uint8_t *)&tmp, offsetof(record_t, crc));
}

/* @return ERR_OK valid, ERR_EMPTY erased slot, ERR_CRC damaged. */
static int read_slot(const datalog_t *log, uint32_t slot, record_t *r)
{
    int rc = log->flash->read(log->flash->ctx, slot_addr(log, slot), r, sizeof(*r));
    if (rc != ERR_OK) {
        return rc;
    }
    if (r->magic == 0xFFFFU) {
        return ERR_EMPTY;
    }
    if (r->magic != REC_MAGIC || record_crc(r) != r->crc) {
        return ERR_CRC;
    }
    return ERR_OK;
}

static uint32_t wrap(const datalog_t *log, uint32_t slot)
{
    return slot % log->total_slots;
}

int datalog_init(datalog_t *log, const flash_dev_t *flash, uint32_t base, uint32_t nsectors)
{
    if (log == NULL || flash == NULL || nsectors < 2U ||
        (base % flash->sector_size) != 0U ||
        base + nsectors * flash->sector_size > flash->size) {
        return ERR_PARAM;
    }
    memset(log, 0, sizeof(*log));
    log->flash            = flash;
    log->base             = base;
    log->nsectors         = nsectors;
    log->slots_per_sector = flash->sector_size / DATALOG_RECORD_SIZE;
    log->total_slots      = log->slots_per_sector * nsectors;
    log->next_seq         = 1U;

    /* 1. Look only at the first record of each sector. The sector holding
     *    the highest sequence number is where writing stopped, the one with
     *    the lowest is the oldest data. */
    bool     found = false;
    uint32_t max_seq = 0U, min_seq = UINT32_MAX, head_sec = 0U, tail_sec = 0U;
    for (uint32_t s = 0U; s < nsectors; s++) {
        record_t r;
        int rc = read_slot(log, s * log->slots_per_sector, &r);
        if (rc == ERR_OK) {
            found = true;
            if (r.seq >= max_seq) { max_seq = r.seq; head_sec = s; }
            if (r.seq <  min_seq) { min_seq = r.seq; tail_sec = s; }
        } else if (rc != ERR_EMPTY && rc != ERR_CRC) {
            return rc;                          /* flash read failed */
        }
    }
    if (!found) {
        return ERR_OK;                          /* empty log */
    }

    /* 2. Inside the newest sector, find the last written slot. */
    uint32_t first = head_sec * log->slots_per_sector;
    uint32_t last  = first;
    for (uint32_t i = 0U; i < log->slots_per_sector; i++) {
        record_t r;
        int rc = read_slot(log, first + i, &r);
        if (rc == ERR_EMPTY) {
            break;
        }
        last = first + i;
        if (rc == ERR_OK && r.seq >= max_seq) {
            max_seq = r.seq;
        }
    }
    log->head     = wrap(log, last + 1U);
    log->tail     = tail_sec * log->slots_per_sector;
    log->count    = (log->head + log->total_slots - log->tail) % log->total_slots;
    if (log->count == 0U) {
        log->count = log->total_slots;          /* completely full ring */
    }
    log->next_seq = max_seq + 1U;
    log->unsent   = log->tail;
    return ERR_OK;
}

int datalog_append(datalog_t *log, const datalog_sample_t *s)
{
    const flash_dev_t *f = log->flash;

    if ((log->head % log->slots_per_sector) == 0U) {
        /* Entering a new sector: erase it. If the oldest data lives there,
         * the tail moves to the next sector (oldest records are dropped). */
        uint32_t sec_first = log->head;
        if (log->count > 0U &&
            (log->tail / log->slots_per_sector) == (sec_first / log->slots_per_sector) &&
            log->count + log->slots_per_sector > log->total_slots) {
            uint32_t dropped = log->slots_per_sector;
            log->tail  = wrap(log, sec_first + log->slots_per_sector);
            log->count = (log->count > dropped) ? log->count - dropped : 0U;
        }
        int rc = f->erase_sector(f->ctx, slot_addr(log, sec_first));
        if (rc != ERR_OK) {
            return rc;
        }
        /* keep the upload cursor inside the stored range */
        uint32_t cur_off = (log->unsent + log->total_slots - log->tail) % log->total_slots;
        if (cur_off >= log->count) {
            log->unsent = log->tail;
        }
    }

    record_t r;
    memset(&r, 0, sizeof r);
    r.magic     = REC_MAGIC;
    r.flags     = FLAG_UNSENT;
    r.version   = REC_VERSION;
    r.seq       = log->next_seq;
    r.timestamp = s->timestamp;
    r.temp_c100 = s->temp_c100;
    r.hum_c100  = s->hum_c100;
    r.rpm       = s->rpm;
    r.duty_c10  = s->duty_c10;
    r.state     = s->state;
    r.faults    = s->faults;
    r.crc       = record_crc(&r);

    int rc = f->write(f->ctx, slot_addr(log, log->head), &r, sizeof r);
    if (rc != ERR_OK) {
        return rc;
    }
    log->next_seq++;
    log->head = wrap(log, log->head + 1U);
    if (log->count < log->total_slots) {
        log->count++;
    }
    return ERR_OK;
}

uint32_t datalog_count(const datalog_t *log)
{
    return log->count;
}

static void to_sample(const record_t *r, datalog_sample_t *s)
{
    s->timestamp = r->timestamp;
    s->temp_c100 = r->temp_c100;
    s->hum_c100  = r->hum_c100;
    s->rpm       = r->rpm;
    s->duty_c10  = r->duty_c10;
    s->state     = r->state;
    s->faults    = r->faults;
}

int datalog_read(datalog_t *log, uint32_t index, datalog_sample_t *s, uint32_t *seq)
{
    if (index >= log->count) {
        return ERR_PARAM;
    }
    record_t r;
    int rc = read_slot(log, wrap(log, log->tail + index), &r);
    if (rc != ERR_OK) {
        return (rc == ERR_EMPTY) ? ERR_CRC : rc;
    }
    to_sample(&r, s);
    if (seq != NULL) {
        *seq = r.seq;
    }
    return ERR_OK;
}

int datalog_next_unsent(datalog_t *log, uint32_t *slot, uint32_t *seq, datalog_sample_t *s)
{
    while (log->unsent != log->head) {
        record_t r;
        int rc = read_slot(log, log->unsent, &r);
        if (rc == ERR_OK && r.flags == FLAG_UNSENT) {
            *slot = log->unsent;
            *seq  = r.seq;
            to_sample(&r, s);
            return ERR_OK;
        }
        if (rc != ERR_OK && rc != ERR_CRC && rc != ERR_EMPTY) {
            return rc;
        }
        log->unsent = wrap(log, log->unsent + 1U);   /* sent or damaged: skip */
    }
    return ERR_EMPTY;
}

int datalog_mark_sent(datalog_t *log, uint32_t slot)
{
    uint8_t flag = FLAG_SENT;
    int rc = log->flash->write(log->flash->ctx, slot_addr(log, slot) + FLAGS_OFFSET, &flag, 1U);
    if (rc == ERR_OK && slot == log->unsent) {
        log->unsent = wrap(log, log->unsent + 1U);
    }
    return rc;
}

int datalog_format(datalog_t *log)
{
    /* Only sectors that hold data need erasing: an empty first slot means
     * the whole sector is still erased (slot 0 is always written first).
     * Untouched sectors are erased later, when the ring reaches them. */
    for (uint32_t s = 0U; s < log->nsectors; s++) {
        uint32_t addr  = log->base + s * log->flash->sector_size;
        uint16_t magic = 0U;
        int rc = log->flash->read(log->flash->ctx, addr, &magic, sizeof magic);
        if (rc == ERR_OK && magic != 0xFFFFU) {
            rc = log->flash->erase_sector(log->flash->ctx, addr);
        }
        if (rc != ERR_OK) {
            return rc;
        }
    }
    log->head = log->tail = log->count = log->unsent = 0U;
    log->next_seq = 1U;
    return ERR_OK;
}
