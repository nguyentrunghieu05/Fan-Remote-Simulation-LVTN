/**
 * @file    fault.h
 * @brief   Fault detection with time qualification.
 *
 * A condition must be true continuously for `trip_ms` before the fault is
 * raised, and false for `clear_ms` before it is released. This filters out
 * glitches (one bad I2C read, a fan that needs a second to spin up).
 * Faults stay latched until fault_clear() if `latching` is set.
 */
#ifndef FAULT_H
#define FAULT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FAULT_FAN_STALL   = 0,  /* output on but tach reads ~0 rpm        */
    FAULT_SENSOR      = 1,  /* temperature sensor not answering        */
    FAULT_OVER_TEMP   = 2,  /* temperature above hard limit            */
    FAULT_STORAGE     = 3,  /* SPI flash missing or failing            */
    FAULT_COUNT
} fault_id_t;

typedef struct {
    uint32_t trip_ms;
    uint32_t clear_ms;
    bool     latching;
    /* state */
    uint32_t timer_ms;
    bool     active;
} fault_item_t;

typedef struct {
    fault_item_t item[FAULT_COUNT];
} fault_monitor_t;

void     fault_init(fault_monitor_t *m);
void     fault_config(fault_monitor_t *m, fault_id_t id, uint32_t trip_ms, uint32_t clear_ms, bool latching);
/** Feed the raw condition each control period. @return true if state changed. */
bool     fault_update(fault_monitor_t *m, fault_id_t id, bool condition, uint32_t dt_ms);
void     fault_clear(fault_monitor_t *m, fault_id_t id);
void     fault_clear_all(fault_monitor_t *m);
bool     fault_active(const fault_monitor_t *m, fault_id_t id);
uint32_t fault_bits(const fault_monitor_t *m);
const char *fault_name(fault_id_t id);

#endif /* FAULT_H */
