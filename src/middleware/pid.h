/**
 * @file    pid.h
 * @brief   Discrete PID controller with output clamping and anti-windup.
 *
 * Used for "auto" mode: keep the measured temperature at the set point by
 * changing the fan duty. Because a fan cools, a temperature ABOVE the set
 * point must give MORE output, so the controller is used in reverse
 * direction (error = measurement - setpoint).
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    bool  reverse;         /* true: output rises when measurement > setpoint */
    /* state */
    float integral;
    float prev_meas;
    bool  first;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min, float out_max, bool reverse);
void  pid_reset(pid_ctrl_t *p);
/** @param dt_s sample time in seconds. @return controller output. */
float pid_update(pid_ctrl_t *p, float setpoint, float measurement, float dt_s);

#endif /* PID_H */
