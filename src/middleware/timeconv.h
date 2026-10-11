/**
 * @file    timeconv.h
 * @brief   Unix time <-> calendar date (UTC), no libc time functions needed.
 *
 * The STM32 RTC counts in calendar format (Y-M-D h:m:s); log records and
 * telemetry use a 32-bit Unix timestamp. Valid for years 2000..2099.
 */
#ifndef TIMECONV_H
#define TIMECONV_H

#include <stdint.h>

typedef struct {
    uint16_t year;   /* 2000..2099 */
    uint8_t  month;  /* 1..12 */
    uint8_t  day;    /* 1..31 */
    uint8_t  hour, minute, second;
    uint8_t  weekday;/* 1 = Monday .. 7 = Sunday */
} datetime_t;

uint32_t datetime_to_unix(const datetime_t *dt);
void     unix_to_datetime(uint32_t t, datetime_t *dt);

#endif /* TIMECONV_H */
