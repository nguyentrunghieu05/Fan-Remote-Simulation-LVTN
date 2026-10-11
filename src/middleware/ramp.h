/**
 * @file    ramp.h
 * @brief   Slew-rate limiter: the fan output never jumps, it ramps.
 *
 * Protects the motor and the supply from inrush current when the speed
 * command changes abruptly (mode change, Modbus write, fault recovery).
 */
#ifndef RAMP_H
#define RAMP_H

typedef struct {
    float value;          /* current output */
    float rate_per_s;     /* max change per second, e.g. 20 %/s */
} ramp_t;

void  ramp_init(ramp_t *r, float start, float rate_per_s);
float ramp_step(ramp_t *r, float target, float dt_s);

#endif /* RAMP_H */
