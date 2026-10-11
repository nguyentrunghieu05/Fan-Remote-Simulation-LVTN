#include "sensors.h"
#include "sht3x.h"
#include "sys_data.h"
#include "supervisor.h"
#include "bsp.h"
#include "err.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#define TICK_MS            10U
#define BUTTON_LONG_MS     2000U    /* long press = clear faults */
#define SHT_PERIOD_MS      1000U
#define SHT_MAX_ERRORS     3U       /* consecutive errors before "invalid" */
#define SHT_RETRY_MS       10000U
#define POT_FILTER         0.2f     /* exponential moving average */

static sht3x_t  s_sht;
static bool     s_sht_present;

void sensors_init(const i2c_bus_t *bus)
{
    s_sht_present = (sht3x_init(&s_sht, bus, SHT3X_ADDR_DEFAULT, bsp_delay_ms) == ERR_OK);
    log_printf("SENS", "SHT3x %s at 0x%02X", s_sht_present ? "found" : "NOT found", SHT3X_ADDR_DEFAULT);
}

static void handle_button(uint32_t *held_ms, bool *stable, uint8_t *count)
{
    bool raw = bsp_button_pressed();
    /* debounce: the level must be the same for 3 samples (30 ms) */
    if (raw != *stable) {
        if (++(*count) >= 3U) {
            *stable = raw;
            *count  = 0U;
            if (!raw) {                                 /* released */
                if (*held_ms < BUTTON_LONG_MS) {
                    (void)app_post_event(APP_EV_BUTTON);
                }
                *held_ms = 0U;
            }
        }
    } else {
        *count = 0U;
    }
    if (*stable) {
        *held_ms += TICK_MS;
        if (*held_ms == BUTTON_LONG_MS) {
            app_request_fault_reset();
            log_printf("SENS", "long press: fault reset");
        }
    }
}

void sensors_task(void *arg)
{
    (void)arg;
    int sv = sv_register("sensors", 1000U);
    TickType_t last = xTaskGetTickCount();
    uint32_t t = 0U, held = 0U, sht_errors = SHT_MAX_ERRORS, retry_at = 0U;
    uint8_t  cnt = 0U;
    bool     btn = false;
    float    pot = 0.0f;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TICK_MS));
        sv_kick(sv);
        t += TICK_MS;

        handle_button(&held, &btn, &cnt);

        if (t % 100U == 0U) {
            float p = bsp_adc_read_pot() * (100.0f / 4095.0f);
            pot += POT_FILTER * (p - pot);
            uint16_t rpm = bsp_tach_rpm();
            sys_lock();
            sys_data()->m.pot_percent = pot;
            sys_data()->m.rpm         = rpm;
            sys_unlock();
        }

        if (t % SHT_PERIOD_MS == 0U) {
            if (!s_sht_present && t >= retry_at) {
                /* hot-plug / recovery: try to find the sensor again */
                s_sht_present = (sht3x_soft_reset(&s_sht) == ERR_OK);
                retry_at = t + SHT_RETRY_MS;
            }
            sht3x_sample_t s;
            int rc = s_sht_present ? sht3x_measure(&s_sht, &s) : ERR_NOT_FOUND;
            sys_lock();
            sys_data_t *d = sys_data();
            if (rc == ERR_OK) {
                sht_errors       = 0U;
                d->m.temp_c      = s.temp_c100 / 100.0f;
                d->m.hum_pct     = s.hum_c100 / 100.0f;
                d->m.temp_valid  = true;
            } else {
                d->m.sensor_errors++;
                if (sht_errors < SHT_MAX_ERRORS) {
                    sht_errors++;
                }
                if (sht_errors >= SHT_MAX_ERRORS) {
                    d->m.temp_valid = false;
                    s_sht_present   = false;
                }
            }
            sys_unlock();
        }
    }
}
