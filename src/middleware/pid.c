#include "pid.h"

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min, float out_max, bool reverse)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->reverse = reverse;
    pid_reset(p);
}

void pid_reset(pid_ctrl_t *p)
{
    p->integral  = 0.0f;
    p->prev_meas = 0.0f;
    p->first     = true;
}

float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt_s)
{
    float error = p->reverse ? (measurement - setpoint) : (setpoint - measurement);

    /* Derivative on measurement, not on error: no "kick" when the user
     * changes the set point. */
    float dmeas = p->first ? 0.0f : (measurement - p->prev_meas) / dt_s;
    if (p->reverse) {
        dmeas = -dmeas;
    }
    p->prev_meas = measurement;
    p->first     = false;

    float pterm = p->kp * error;
    float dterm = -p->kd * dmeas;

    /* Conditional integration (anti-windup): only integrate while the
     * output is not saturated in the direction the error pushes. */
    float candidate = p->integral + p->ki * error * dt_s;
    float out       = pterm + candidate + dterm;
    if ((out > p->out_max && error > 0.0f) || (out < p->out_min && error < 0.0f)) {
        out = pterm + p->integral + dterm;     /* keep previous integral */
    } else {
        p->integral = candidate;
    }
    return clampf(out, p->out_min, p->out_max);
}
