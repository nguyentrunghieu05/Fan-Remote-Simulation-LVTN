#include "sensors.h"
#include "sht3x.h"
#include "bme280.h"
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

/* The temperature sensor is found at run time, so the same firmware works
 * with whatever module is on the bus: SHT3x first, then BME280 / BMP280. */
typedef enum { TS_NONE = 0, TS_SHT3X, TS_BME280 } temp_sensor_t;

static const i2c_bus_t *s_bus;
static temp_sensor_t    s_kind;
static sht3x_t          s_sht;
static bme280_t         s_bme;

typedef struct {
    int16_t  temp_c100;
    uint16_t hum_c100;
    uint32_t press_pa;
} temp_sample_t;

static const char *kind_name(void)
{
    switch (s_kind) {
    case TS_SHT3X:  return "SHT3x";
    case TS_BME280: return s_bme.has_humidity ? "BME280" : "BMP280";
    default:        return "none";
    }
}

static temp_sensor_t detect(void)
{
    static const uint8_t sht_addr[] = { SHT3X_ADDR_DEFAULT, SHT3X_ADDR_ALT };
    static const uint8_t bme_addr[] = { BME280_ADDR_DEFAULT, BME280_ADDR_ALT };
    for (unsigned i = 0U; i < sizeof sht_addr; i++) {
        if (sht3x_init(&s_sht, s_bus, sht_addr[i], bsp_delay_ms) == ERR_OK) {
            return TS_SHT3X;
        }
    }
    for (unsigned i = 0U; i < sizeof bme_addr; i++) {
        if (bme280_init(&s_bme, s_bus, bme_addr[i], bsp_delay_ms) == ERR_OK) {
            return TS_BME280;
        }
    }
    return TS_NONE;
}

static int measure(temp_sample_t *out)
{
    int rc = ERR_NOT_FOUND;
    if (s_kind == TS_SHT3X) {
        sht3x_sample_t s;
        rc = sht3x_measure(&s_sht, &s);
        out->temp_c100 = s.temp_c100;
        out->hum_c100  = s.hum_c100;
        out->press_pa  = 0U;
    } else if (s_kind == TS_BME280) {
        bme280_sample_t s;
        rc = bme280_measure(&s_bme, &s);
        out->temp_c100 = s.temp_c100;
        out->hum_c100  = s.hum_c100;
        out->press_pa  = s.press_pa;
    }
    return rc;
}

static void publish_sensor_name(void)
{
    sys_lock();
    sys_data()->m.sensor_name = kind_name();
    sys_unlock();
}

void sensors_init(const i2c_bus_t *bus)
{
    s_bus  = bus;
    s_kind = detect();
    if (s_kind == TS_SHT3X) {
        log_printf("SENS", "SHT3x found at 0x%02X", s_sht.addr);
    } else if (s_kind == TS_BME280) {
        log_printf("SENS", "%s found at 0x%02X (id 0x%02X)", kind_name(), s_bme.addr, s_bme.chip_id);
    } else {
        log_printf("SENS", "no temperature sensor (SHT3x 0x44/45, BME280 0x76/77)");
    }
    publish_sensor_name();
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
            if (s_kind == TS_NONE && t >= retry_at) {
                /* hot-plug / recovery: look for a sensor again */
                s_kind = detect();
                retry_at = t + SHT_RETRY_MS;
                if (s_kind != TS_NONE) {
                    log_printf("SENS", "%s found", kind_name());
                    publish_sensor_name();
                }
            }
            temp_sample_t s;
            int rc = (s_kind != TS_NONE) ? measure(&s) : ERR_NOT_FOUND;
            sys_lock();
            sys_data_t *d = sys_data();
            if (rc == ERR_OK) {
                sht_errors       = 0U;
                d->m.temp_c      = s.temp_c100 / 100.0f;
                d->m.hum_pct     = s.hum_c100 / 100.0f;
                d->m.press_hpa   = s.press_pa / 100.0f;
                d->m.temp_valid  = true;
            } else {
                d->m.sensor_errors++;
                if (sht_errors < SHT_MAX_ERRORS) {
                    sht_errors++;
                }
                if (sht_errors >= SHT_MAX_ERRORS && s_kind != TS_NONE) {
                    d->m.temp_valid  = false;
                    d->m.sensor_name = "none";
                    s_kind = TS_NONE;
                } else if (sht_errors >= SHT_MAX_ERRORS) {
                    d->m.temp_valid = false;
                }
            }
            sys_unlock();
        }
    }
}
