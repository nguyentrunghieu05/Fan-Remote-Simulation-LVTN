#include "timeconv.h"

static int is_leap(int y) { return ((y % 4) == 0 && (y % 100) != 0) || (y % 400) == 0; }

static const uint8_t mdays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static int days_in_month(int y, int m)
{
    return (m == 2 && is_leap(y)) ? 29 : mdays[m - 1];
}

uint32_t datetime_to_unix(const datetime_t *dt)
{
    uint32_t days = 0U;
    for (int y = 1970; y < dt->year; y++) {
        days += is_leap(y) ? 366U : 365U;
    }
    for (int m = 1; m < dt->month; m++) {
        days += (uint32_t)days_in_month(dt->year, m);
    }
    days += (uint32_t)(dt->day - 1U);
    return days * 86400UL + dt->hour * 3600UL + dt->minute * 60UL + dt->second;
}

void unix_to_datetime(uint32_t t, datetime_t *dt)
{
    uint32_t days = t / 86400UL;
    uint32_t rem  = t % 86400UL;
    dt->hour   = (uint8_t)(rem / 3600UL);
    dt->minute = (uint8_t)((rem % 3600UL) / 60UL);
    dt->second = (uint8_t)(rem % 60UL);
    /* 1970-01-01 was a Thursday (ISO weekday 4). */
    dt->weekday = (uint8_t)(((days + 3U) % 7U) + 1U);

    int y = 1970;
    for (;;) {
        uint32_t ylen = is_leap(y) ? 366U : 365U;
        if (days < ylen) {
            break;
        }
        days -= ylen;
        y++;
    }
    int m = 1;
    while (days >= (uint32_t)days_in_month(y, m)) {
        days -= (uint32_t)days_in_month(y, m);
        m++;
    }
    dt->year  = (uint16_t)y;
    dt->month = (uint8_t)m;
    dt->day   = (uint8_t)(days + 1U);
}
