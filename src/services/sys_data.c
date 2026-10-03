#include "sys_data.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

static sys_data_t        s_data;
static SemaphoreHandle_t s_lock;

void sys_data_init(const fan_config_t *cfg)
{
    memset(&s_data, 0, sizeof s_data);
    s_data.cfg     = *cfg;
    s_data.m.state = APP_STATE_OFF;
    s_lock = xSemaphoreCreateMutex();
}

void sys_lock(void)       { xSemaphoreTake(s_lock, portMAX_DELAY); }
void sys_unlock(void)     { xSemaphoreGive(s_lock); }
sys_data_t *sys_data(void) { return &s_data; }

void sys_snapshot(sys_data_t *out)
{
    sys_lock();
    *out = s_data;
    sys_unlock();
}

const char *fmt_fixed(char *buf, int32_t value, int decimals)
{
    /* buf must hold at least 16 characters */
    const char *sign = (value < 0) ? "-" : "";
    unsigned long a = (unsigned long)((value < 0) ? -(int64_t)value : value);
    if (decimals == 2) {
        snprintf(buf, 16, "%s%lu.%02lu", sign, a / 100UL, a % 100UL);
    } else if (decimals == 1) {
        snprintf(buf, 16, "%s%lu.%lu", sign, a / 10UL, a % 10UL);
    } else {
        snprintf(buf, 16, "%s%lu", sign, a);
    }
    return buf;
}
