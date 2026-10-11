/**
 * @file    sys_data.h
 * @brief   Shared system data (measurements + settings) and app events.
 *
 * One struct, one mutex. Tasks copy what they need under the lock
 * (sys_snapshot) and write only their own fields. Mode changes are not
 * written directly: they are posted as events to the control task, which
 * owns the state machine. This keeps every state change in ONE place.
 */
#ifndef SYS_DATA_H
#define SYS_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "app_fsm.h"
#include "fan_logic.h"

typedef struct {
    /* written by the sensor task */
    float       temp_c;
    float       hum_pct;
    float       press_hpa;     /* 0 when the sensor has no barometer */
    const char *sensor_name;   /* "SHT3x", "BME280", "BMP280" or "none" */
    bool        temp_valid;
    uint16_t    rpm;
    float       pot_percent;
    uint32_t    sensor_errors;
    /* written by the control task */
    float       duty;
    app_state_t state;
    uint32_t    faults;
    /* written at boot */
    bool        flash_ok;
    bool        oled_ok;       /* a display (OLED or LCD) answered */
    const char *display_name;  /* "SSD1306", "LCD1602" or "none" */
} sys_meas_t;

typedef struct {
    sys_meas_t   m;
    fan_config_t cfg;
} sys_data_t;

void sys_data_init(const fan_config_t *cfg);
void sys_lock(void);
void sys_unlock(void);
/** Direct access, only between sys_lock() and sys_unlock(). */
sys_data_t *sys_data(void);
/** Consistent copy of everything. */
void sys_snapshot(sys_data_t *out);

/** Ask the control task to change mode (thread safe, never blocks long). */
bool app_post_event(app_event_t ev);
/** Ask the control task to clear latched faults. */
void app_request_fault_reset(void);

/** Fixed-point formatting without printf("%f") (no float printf on newlib-nano).
 *  buf must hold 16 characters. */
const char *fmt_fixed(char *buf, int32_t value, int decimals);

#endif /* SYS_DATA_H */
