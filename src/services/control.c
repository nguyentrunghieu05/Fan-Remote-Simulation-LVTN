/**
 * @file    control.c
 * @brief   The 100 ms control loop - the only owner of the fan output.
 *
 *   events (button, CLI, Modbus, LoRa) --> queue --> app_fsm
 *   measurements --> fault monitor --> FAULT_RAISED / FAULT_CLEARED
 *   state + measurements --> fan_logic (PID / manual / fail-safe + ramp)
 *   --> PWM + DAC, relay, LED
 */
#include "control.h"
#include "sys_data.h"
#include "supervisor.h"
#include "fault.h"
#include "bsp.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Faults that force the FAULT state (fail-safe output + alarm relay).
 * A missing SPI flash is only a warning: control does not depend on it. */
#define CRITICAL_FAULTS  ((1UL << FAULT_FAN_STALL) | (1UL << FAULT_SENSOR) | (1UL << FAULT_OVER_TEMP))

static QueueHandle_t     s_events;
static volatile bool     s_fault_reset;
static fault_monitor_t   s_faults;
static app_fsm_t         s_fsm;
static fan_logic_t       s_logic;

bool app_post_event(app_event_t ev)
{
    return xQueueSend(s_events, &ev, pdMS_TO_TICKS(10)) == pdTRUE;
}

void app_request_fault_reset(void)
{
    s_fault_reset = true;
}

void control_init(void)
{
    s_events = xQueueCreate(8, sizeof(app_event_t));
    fault_init(&s_faults);
    /*                          id               trip    clear  latching */
    fault_config(&s_faults, FAULT_FAN_STALL,     5000U,     0U, true);
    fault_config(&s_faults, FAULT_SENSOR,        3000U,  2000U, false);
    fault_config(&s_faults, FAULT_OVER_TEMP,     2000U,  5000U, false);
    fault_config(&s_faults, FAULT_STORAGE,          0U,  1000U, false);
}

static void update_faults(const sys_data_t *d)
{
    const fan_config_t *c = &d->cfg;
    bool stall = c->stall_rpm > 0U && d->m.duty >= (c->min_duty_c10 / 10.0f) && d->m.duty > 5.0f &&
                 d->m.rpm < c->stall_rpm;
    /* Over-temperature with 2 °C hysteresis. */
    float limit = c->over_temp_c100 / 100.0f;
    if (fault_active(&s_faults, FAULT_OVER_TEMP)) {
        limit -= 2.0f;
    }
    bool hot = d->m.temp_valid && d->m.temp_c > limit;
    /* The sensor only matters when the controller uses it (AUTO). */
    bool sensor = !d->m.temp_valid && (s_fsm.state == APP_STATE_AUTO ||
                                       (s_fsm.state == APP_STATE_FAULT && s_fsm.resume == APP_STATE_AUTO));

    uint32_t before = fault_bits(&s_faults);
    fault_update(&s_faults, FAULT_FAN_STALL, stall,  CONTROL_PERIOD_MS);
    fault_update(&s_faults, FAULT_SENSOR,    sensor, CONTROL_PERIOD_MS);
    fault_update(&s_faults, FAULT_OVER_TEMP, hot,    CONTROL_PERIOD_MS);
    fault_update(&s_faults, FAULT_STORAGE,   !d->m.flash_ok, CONTROL_PERIOD_MS);
    uint32_t after = fault_bits(&s_faults);

    for (int i = 0; i < FAULT_COUNT; i++) {
        uint32_t bit = 1UL << i;
        if ((before ^ after) & bit) {
            log_printf("FAULT", "%s %s", fault_name((fault_id_t)i), (after & bit) ? "RAISED" : "cleared");
        }
    }
}

static void drive_led(app_state_t st, uint32_t tick)
{
    switch (st) {
    case APP_STATE_AUTO:   bsp_led_set(true); break;                      /* steady   */
    case APP_STATE_MANUAL: bsp_led_set((tick / 5U) % 2U == 0U); break;    /* 1 Hz     */
    case APP_STATE_FAULT:  bsp_led_set(tick % 2U == 0U); break;           /* 5 Hz     */
    case APP_STATE_OFF:
    default:               bsp_led_set(tick % 20U == 0U); break;          /* short blip every 2 s */
    }
}

void control_task(void *arg)
{
    (void)arg;
    sys_data_t d;
    sys_snapshot(&d);
    app_fsm_init(&s_fsm, (app_state_t)d.cfg.start_mode);
    fan_logic_init(&s_logic, &d.cfg);
    log_printf("CTRL", "start in %s", app_state_name(s_fsm.state));

    int sv = sv_register("control", 500U);
    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0U;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        sv_kick(sv);
        tick++;

        /* 1. commands */
        app_event_t ev;
        while (xQueueReceive(s_events, &ev, 0) == pdTRUE) {
            if (app_fsm_handle(&s_fsm, ev)) {
                log_printf("CTRL", "-> %s", app_state_name(s_fsm.state));
            }
        }
        if (s_fault_reset) {
            s_fault_reset = false;
            fault_clear_all(&s_faults);
            log_printf("FAULT", "faults cleared by user");
        }

        /* 2. faults */
        sys_snapshot(&d);
        update_faults(&d);
        uint32_t crit = fault_bits(&s_faults) & CRITICAL_FAULTS;
        if (crit != 0U && s_fsm.state != APP_STATE_FAULT) {
            (void)app_fsm_handle(&s_fsm, APP_EV_FAULT_RAISED);
            log_printf("CTRL", "-> FAULT (mask 0x%02lX), output %u.%u %%",
                       (unsigned long)crit, d.cfg.fault_duty_c10 / 10U, d.cfg.fault_duty_c10 % 10U);
        } else if (crit == 0U && s_fsm.state == APP_STATE_FAULT) {
            (void)app_fsm_handle(&s_fsm, APP_EV_FAULT_CLEARED);
            log_printf("CTRL", "fault gone -> %s", app_state_name(s_fsm.state));
        }

        /* 3. control law and outputs */
        fan_inputs_t in = {
            .temp_c      = d.m.temp_c,
            .temp_valid  = d.m.temp_valid,
            .rpm         = d.m.rpm,
            .pot_percent = d.m.pot_percent,
        };
        float duty = fan_logic_step(&s_logic, &d.cfg, s_fsm.state, &in, CONTROL_PERIOD_MS / 1000.0f);
        bsp_fan_out_set(duty);
        bsp_relay_set(crit != 0U);
        drive_led(s_fsm.state, tick);

        /* 4. publish */
        sys_lock();
        sys_data()->m.duty   = duty;
        sys_data()->m.state  = s_fsm.state;
        sys_data()->m.faults = fault_bits(&s_faults);
        sys_unlock();
    }
}
