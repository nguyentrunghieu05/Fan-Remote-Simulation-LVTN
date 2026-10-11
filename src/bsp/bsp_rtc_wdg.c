/**
 * @file    bsp_rtc_wdg.c
 * @brief   Real-time clock (timestamps for the log) and independent watchdog.
 *
 * RTC clock: LSE 32.768 kHz crystal (fitted on the Nucleo, X2). If the
 * crystal does not start, LSI (~32 kHz, less accurate) is used instead.
 * Backup register DR0 marks whether the time has been set since power-up,
 * DR1 counts resets caused by the independent watchdog.
 */
#include "bsp.h"
#include "board.h"
#include "timeconv.h"
#include <string.h>

#define RTC_MAGIC      0x32F2U
#define WDT_COUNT_REG  RTC_BKP_DR1     /* number of IWDG resets */

static RTC_HandleTypeDef  s_rtc;
static IWDG_HandleTypeDef s_iwdg;

void bsp_rtc_init(void)
{
    RCC_OscInitTypeDef       osc = { 0 };
    RCC_PeriphCLKInitTypeDef pc  = { 0 };
    uint32_t src = RCC_RTCCLKSOURCE_LSE;

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.LSEState       = RCC_LSE_ON;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        osc.OscillatorType = RCC_OSCILLATORTYPE_LSI;
        osc.LSIState       = RCC_LSI_ON;
        (void)HAL_RCC_OscConfig(&osc);
        src = RCC_RTCCLKSOURCE_LSI;
    }
    pc.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    pc.RTCClockSelection    = src;
    (void)HAL_RCCEx_PeriphCLKConfig(&pc);
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();

    s_rtc.Instance            = RTC;
    s_rtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    s_rtc.Init.AsynchPrediv   = 127U;                /* 32768 / 128 / 256 = 1 Hz */
    s_rtc.Init.SynchPrediv    = (src == RCC_RTCCLKSOURCE_LSE) ? 255U : 249U;  /* LSI = 32 kHz */
    s_rtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    s_rtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    s_rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    s_rtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    s_rtc.Init.OutPutPullUp   = RTC_OUTPUT_PULLUP_NONE;
    (void)HAL_RTC_Init(&s_rtc);

    /* bsp_clock_init() has already read the reset flags. */
    if (strcmp(bsp_reset_cause(), "IWDG") == 0) {
        HAL_RTCEx_BKUPWrite(&s_rtc, WDT_COUNT_REG, HAL_RTCEx_BKUPRead(&s_rtc, WDT_COUNT_REG) + 1U);
    }
}

uint32_t bsp_wdt_reset_count(void)
{
    return HAL_RTCEx_BKUPRead(&s_rtc, WDT_COUNT_REG);
}

void bsp_wdt_reset_count_clear(void)
{
    HAL_RTCEx_BKUPWrite(&s_rtc, WDT_COUNT_REG, 0U);
}

bool bsp_rtc_is_set(void)
{
    return HAL_RTCEx_BKUPRead(&s_rtc, RTC_BKP_DR0) == RTC_MAGIC;
}

uint32_t bsp_rtc_get_unix(void)
{
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;
    /* Time first, then date: reading the date unlocks the shadow registers. */
    (void)HAL_RTC_GetTime(&s_rtc, &t, RTC_FORMAT_BIN);
    (void)HAL_RTC_GetDate(&s_rtc, &d, RTC_FORMAT_BIN);
    datetime_t dt = { (uint16_t)(2000U + d.Year), d.Month, d.Date, t.Hours, t.Minutes, t.Seconds, d.WeekDay };
    return datetime_to_unix(&dt);
}

void bsp_rtc_set_unix(uint32_t ts)
{
    datetime_t dt;
    unix_to_datetime(ts, &dt);
    RTC_TimeTypeDef t = { 0 };
    RTC_DateTypeDef d = { 0 };
    t.Hours   = dt.hour;
    t.Minutes = dt.minute;
    t.Seconds = dt.second;
    d.Year    = (uint8_t)(dt.year - 2000U);
    d.Month   = dt.month;
    d.Date    = dt.day;
    d.WeekDay = dt.weekday;
    (void)HAL_RTC_SetTime(&s_rtc, &t, RTC_FORMAT_BIN);
    (void)HAL_RTC_SetDate(&s_rtc, &d, RTC_FORMAT_BIN);
    HAL_RTCEx_BKUPWrite(&s_rtc, RTC_BKP_DR0, RTC_MAGIC);
}

void bsp_iwdg_init(uint32_t timeout_ms)
{
    /* LSI 32 kHz / 64 = 500 Hz -> 2 ms per count, 12-bit reload */
    uint32_t reload = timeout_ms / 2U;
    if (reload > 0xFFFU) {
        reload = 0xFFFU;
    }
    s_iwdg.Instance       = IWDG;
    s_iwdg.Init.Prescaler = IWDG_PRESCALER_64;
    s_iwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    s_iwdg.Init.Reload    = reload;
    (void)HAL_IWDG_Init(&s_iwdg);
}

void bsp_iwdg_refresh(void)
{
    (void)HAL_IWDG_Refresh(&s_iwdg);
}
