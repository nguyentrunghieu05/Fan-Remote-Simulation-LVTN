/**
 * @file    power.h
 * @brief   Supply monitor: MCU supply (VDD via VREFINT) and 12 V fan supply.
 *
 * Sampled every 500 ms by the sensor task, which owns the ADC. A sag must
 * last two samples before it is reported, and the warning clears only above
 * a higher level (hysteresis), so a noisy supply does not flood the log.
 *
 *   VDD  low < 3.00 V, ok again > 3.10 V
 *   12 V low < 10.8 V (-10 %), ok again > 11.2 V, "not connected" < 3 V
 */
#ifndef POWER_H
#define POWER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t vdd_mv;
    uint16_t vdd_min_mv;      /* lowest value since boot */
    bool     vdd_low;
    uint32_t vbus_mv;
    uint32_t vbus_min_mv;     /* lowest value while connected */
    bool     vbus_present;
    bool     vbus_low;
    uint32_t sags;            /* number of low-voltage warnings raised */
} power_status_t;

/** Take one measurement and update the warnings. Sensor task only. */
void power_sample(void);
/** Consistent copy for the console. */
void power_get(power_status_t *out);

#endif /* POWER_H */
