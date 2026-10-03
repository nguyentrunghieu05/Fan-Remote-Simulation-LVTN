#include "fan_logic.h"
#include <string.h>

#define CONFIG_VERSION  1U

void fan_config_defaults(fan_config_t *c)
{
    memset(c, 0, sizeof *c);
    c->version         = CONFIG_VERSION;
    c->modbus_addr     = 1U;
    c->start_mode      = APP_STATE_AUTO;
    c->manual_duty_c10 = 500U;      /* 50 %   */
    c->setpoint_c100   = 3000;      /* 30 °C  */
    c->min_duty_c10    = 200U;      /* 20 %   */
    c->fault_duty_c10  = 1000U;     /* 100 %: fail safe = full cooling */
    c->over_temp_c100  = 6000;      /* 60 °C  */
    c->ramp_c10_per_s  = 200U;      /* 20 %/s */
    c->stall_rpm       = 0U;        /* 0 = off; set e.g. 300 when a tach wire is connected */
    c->log_period_s    = 10U;
    c->tlm_period_s    = 30U;
    c->node_id         = 1U;
    c->kp              = 8.0f;
    c->ki              = 0.4f;
    c->kd              = 0.0f;
    c->use_pot         = 1U;
}

#define CLAMP(v, lo, hi)  do { if ((v) < (lo)) { (v) = (lo); changed = true; } \
                               else if ((v) > (hi)) { (v) = (hi); changed = true; } } while (0)

bool fan_config_sanitize(fan_config_t *c)
{
    bool changed = false;
    if (c->version != CONFIG_VERSION) {
        fan_config_defaults(c);
        return true;
    }
    CLAMP(c->modbus_addr, 1U, 247U);
    if (c->start_mode >= APP_STATE_FAULT) { c->start_mode = APP_STATE_OFF; changed = true; }
    CLAMP(c->manual_duty_c10, 0U, 1000U);
    CLAMP(c->setpoint_c100, 0, 8000);
    CLAMP(c->min_duty_c10, 0U, 500U);
    CLAMP(c->fault_duty_c10, 0U, 1000U);
    CLAMP(c->over_temp_c100, 2000, 12000);
    CLAMP(c->ramp_c10_per_s, 10U, 1000U);
    CLAMP(c->log_period_s, 1U, 3600U);
    CLAMP(c->tlm_period_s, 5U, 3600U);
    CLAMP(c->kp, 0.0f, 100.0f);
    CLAMP(c->ki, 0.0f, 50.0f);
    CLAMP(c->kd, 0.0f, 50.0f);
    return changed;
}

void fan_logic_init(fan_logic_t *l, const fan_config_t *c)
{
    pid_init(&l->pid, c->kp, c->ki, c->kd, 0.0f, 100.0f, true);
    ramp_init(&l->ramp, 0.0f, c->ramp_c10_per_s / 10.0f);
    l->last_state = APP_STATE_OFF;
}

float fan_logic_step(fan_logic_t *l, const fan_config_t *c, app_state_t st,
                     const fan_inputs_t *in, float dt_s)
{
    float target;
    float min_duty = c->min_duty_c10 / 10.0f;

    /* Gains may have been changed from the CLI or Modbus. */
    l->pid.kp = c->kp;
    l->pid.ki = c->ki;
    l->pid.kd = c->kd;
    l->ramp.rate_per_s = c->ramp_c10_per_s / 10.0f;

    if (st != l->last_state) {
        if (st == APP_STATE_AUTO) {
            pid_reset(&l->pid);
            /* bumpless transfer: start the integrator at the current output */
            l->pid.integral = l->ramp.value;
        }
        l->last_state = st;
    }

    switch (st) {
    case APP_STATE_MANUAL:
        target = c->use_pot ? in->pot_percent : c->manual_duty_c10 / 10.0f;
        break;
    case APP_STATE_AUTO:
        if (in->temp_valid) {
            target = pid_update(&l->pid, c->setpoint_c100 / 100.0f, in->temp_c, dt_s);
        } else {
            target = c->fault_duty_c10 / 10.0f;   /* no sensor: be safe */
        }
        break;
    case APP_STATE_FAULT:
        target = c->fault_duty_c10 / 10.0f;
        break;
    case APP_STATE_OFF:
    default:
        target = 0.0f;
        break;
    }

    /* A running fan below its minimum duty would stall: either off or >= min. */
    if (target > 0.5f && target < min_duty) {
        target = min_duty;
    } else if (target <= 0.5f) {
        target = 0.0f;
    }
    if (target > 100.0f) {
        target = 100.0f;
    }
    return ramp_step(&l->ramp, target, dt_s);
}
