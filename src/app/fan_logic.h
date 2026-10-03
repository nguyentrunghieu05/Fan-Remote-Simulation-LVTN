/**
 * @file    fan_logic.h
 * @brief   Control law: from state + measurements to a fan duty target.
 *
 * Kept free of RTOS/HAL so it can be tested on a PC with simulated
 * temperature profiles.
 */
#ifndef FAN_LOGIC_H
#define FAN_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "app_fsm.h"
#include "pid.h"
#include "ramp.h"
#include "fault.h"

/** Settings that the user can change and that are saved in flash. */
typedef struct {
    uint16_t version;
    uint8_t  modbus_addr;       /* 1..247                          */
    uint8_t  start_mode;        /* app_state_t after power-up      */
    uint16_t manual_duty_c10;   /* 0..1000 = 0..100.0 %            */
    int16_t  setpoint_c100;     /* auto mode target temperature    */
    uint16_t min_duty_c10;      /* below this the fan would stall  */
    uint16_t fault_duty_c10;    /* output in FAULT state (fail-safe) */
    int16_t  over_temp_c100;    /* hard limit -> OVER_TEMP fault   */
    uint16_t ramp_c10_per_s;    /* slew rate                       */
    uint16_t stall_rpm;         /* below = stalled                 */
    uint16_t log_period_s;      /* data log interval               */
    uint16_t tlm_period_s;      /* telemetry interval              */
    uint16_t node_id;           /* telemetry node id               */
    float    kp, ki, kd;        /* PID gains, in %/°C              */
    uint8_t  use_pot;           /* manual duty follows the potentiometer */
    uint8_t  pad[3];
} fan_config_t;

typedef struct {
    float    temp_c;
    bool     temp_valid;
    uint16_t rpm;
    float    pot_percent;       /* 0..100 from the ADC */
} fan_inputs_t;

typedef struct {
    pid_ctrl_t pid;
    ramp_t     ramp;
    app_state_t last_state;
} fan_logic_t;

void  fan_config_defaults(fan_config_t *c);
/** Clamp every field to a safe range. @return true if something was changed. */
bool  fan_config_sanitize(fan_config_t *c);

void  fan_logic_init(fan_logic_t *l, const fan_config_t *c);
/** @return duty to apply now, percent 0..100 (already ramped). */
float fan_logic_step(fan_logic_t *l, const fan_config_t *c, app_state_t st,
                     const fan_inputs_t *in, float dt_s);

#endif /* FAN_LOGIC_H */
