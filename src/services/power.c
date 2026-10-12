#include "power.h"
#include "bsp.h"
#include "log.h"
#include "sys_data.h"
#include "FreeRTOS.h"
#include "task.h"

#define VDD_LOW_MV        3000U
#define VDD_OK_MV         3100U
#define VBUS_LOW_MV       10800U
#define VBUS_OK_MV        11200U
/* Presence with hysteresis: a floating divider input reads 2..4 V of noise,
 * which must never be taken for a 12 V supply. */
#define VBUS_PRESENT_MV   8000U
#define VBUS_ABSENT_MV    5000U
#define CONFIRM_SAMPLES   2U

static power_status_t s_pwr = { .vdd_min_mv = 0xFFFFU, .vbus_min_mv = 0xFFFFFFFFUL };
static uint8_t        s_vdd_cnt, s_vbus_cnt, s_present_cnt;

/* Low/ok decision with hysteresis and confirmation over a few samples. */
static bool check(bool low_now, bool *flag, uint8_t *cnt, uint32_t mv, uint32_t ok_mv)
{
    if (!*flag) {
        *cnt = low_now ? (uint8_t)(*cnt + 1U) : 0U;
        if (*cnt >= CONFIRM_SAMPLES) {
            *flag = true;
            *cnt  = 0U;
            return true;
        }
    } else if (mv > ok_mv) {
        *flag = false;
        *cnt  = 0U;
        return true;
    }
    return false;
}

void power_sample(void)
{
    char a[16];
    uint16_t vdd  = bsp_adc_vdd_mv();
    uint32_t vbus = bsp_adc_vbus_mv();

    taskENTER_CRITICAL();
    power_status_t p = s_pwr;
    taskEXIT_CRITICAL();

    p.vdd_mv = vdd;
    if (vdd < p.vdd_min_mv) {
        p.vdd_min_mv = vdd;
    }
    if (check(vdd < VDD_LOW_MV, &p.vdd_low, &s_vdd_cnt, vdd, VDD_OK_MV)) {
        if (p.vdd_low) {
            p.sags++;
        }
        log_printf("PWR", "VDD %s: %s V", p.vdd_low ? "low" : "back to normal", fmt_fixed(a, vdd / 10, 2));
    }

    bool change = p.vbus_present ? (vbus < VBUS_ABSENT_MV) : (vbus > VBUS_PRESENT_MV);
    s_present_cnt = change ? (uint8_t)(s_present_cnt + 1U) : 0U;
    if (s_present_cnt >= CONFIRM_SAMPLES) {
        s_present_cnt  = 0U;
        p.vbus_present = !p.vbus_present;
        p.vbus_low     = false;
        s_vbus_cnt     = 0U;
        log_printf("PWR", "12V supply %s (%s V)", p.vbus_present ? "connected" : "not connected",
                   fmt_fixed(a, (int32_t)(vbus / 10U), 2));
    }
    p.vbus_mv = vbus;
    if (p.vbus_present) {
        if (vbus < p.vbus_min_mv) {
            p.vbus_min_mv = vbus;
        }
        if (check(vbus < VBUS_LOW_MV, &p.vbus_low, &s_vbus_cnt, vbus, VBUS_OK_MV)) {
            if (p.vbus_low) {
                p.sags++;
            }
            log_printf("PWR", "12V %s: %s V", p.vbus_low ? "low" : "back to normal",
                       fmt_fixed(a, (int32_t)(vbus / 10U), 2));
        }
    }

    taskENTER_CRITICAL();
    s_pwr = p;
    taskEXIT_CRITICAL();
}

void power_get(power_status_t *out)
{
    taskENTER_CRITICAL();
    *out = s_pwr;
    taskEXIT_CRITICAL();
}
