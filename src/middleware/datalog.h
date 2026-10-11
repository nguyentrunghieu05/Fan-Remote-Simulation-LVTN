/**
 * @file    datalog.h
 * @brief   Circular measurement log on NOR flash (SPI flash W25Qxx).
 *
 * Layout: the log owns `nsectors` erase sectors. Each sector holds
 * sector_size / 32 fixed-size records. Records are appended in order; when
 * the write position reaches a new sector, that sector is erased first, so
 * the oldest data is overwritten (ring behaviour).
 *
 * Every record has a sequence number and a CRC, so after a reset the head
 * and tail are found again by reading only the first record of each sector
 * (fast boot even with 8 MB of flash), and a record half-written during a
 * power loss is simply detected and skipped.
 *
 * "Uploaded" marker: NOR flash can turn bits 1 -> 0 without an erase. Each
 * record has a flags byte written as 0xFF; once the record has been sent to
 * the server it is re-programmed to 0x00. That is how unsent data survives
 * a power cut and is replayed later.
 */
#ifndef DATALOG_H
#define DATALOG_H

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

#define DATALOG_RECORD_SIZE  32U

typedef struct {
    uint32_t timestamp;     /* unix time, s          */
    int16_t  temp_c100;     /* temperature, 0.01 °C  */
    uint16_t hum_c100;      /* humidity, 0.01 %RH    */
    uint16_t rpm;           /* fan speed             */
    uint16_t duty_c10;      /* output, 0.1 %         */
    uint8_t  state;         /* application state     */
    uint8_t  faults;        /* fault bit mask        */
} datalog_sample_t;

typedef struct {
    const flash_dev_t *flash;
    uint32_t base;          /* first byte of the log area (sector aligned) */
    uint32_t nsectors;
    uint32_t slots_per_sector;
    uint32_t total_slots;
    /* ring state, in slot indexes */
    uint32_t head;          /* next slot to write         */
    uint32_t tail;          /* oldest slot still stored    */
    uint32_t count;         /* slots used between tail and head */
    uint32_t next_seq;
    uint32_t unsent;        /* upload cursor (slot index)  */
} datalog_t;

int  datalog_init(datalog_t *log, const flash_dev_t *flash, uint32_t base, uint32_t nsectors);
int  datalog_append(datalog_t *log, const datalog_sample_t *s);
/** Read the i-th record counted from the oldest. ERR_CRC if that slot is damaged. */
int  datalog_read(datalog_t *log, uint32_t index, datalog_sample_t *s, uint32_t *seq);
uint32_t datalog_count(const datalog_t *log);
/** Find the oldest record not yet uploaded. @return ERR_EMPTY if none. */
int  datalog_next_unsent(datalog_t *log, uint32_t *slot, uint32_t *seq, datalog_sample_t *s);
int  datalog_mark_sent(datalog_t *log, uint32_t slot);
/** Erase every sector of the log area that holds data. */
int  datalog_format(datalog_t *log);

#endif /* DATALOG_H */
