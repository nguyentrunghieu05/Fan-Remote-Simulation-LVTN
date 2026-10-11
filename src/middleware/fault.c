#include "fault.h"
#include <string.h>

void fault_init(fault_monitor_t *m)
{
    memset(m, 0, sizeof(*m));
}

void fault_config(fault_monitor_t *m, fault_id_t id, uint32_t trip_ms, uint32_t clear_ms, bool latching)
{
    if (id >= FAULT_COUNT) {
        return;
    }
    m->item[id].trip_ms  = trip_ms;
    m->item[id].clear_ms = clear_ms;
    m->item[id].latching = latching;
}

bool fault_update(fault_monitor_t *m, fault_id_t id, bool condition, uint32_t dt_ms)
{
    if (id >= FAULT_COUNT) {
        return false;
    }
    fault_item_t *f = &m->item[id];

    if (!f->active) {
        f->timer_ms = condition ? f->timer_ms + dt_ms : 0U;
        if (condition && f->timer_ms >= f->trip_ms) {
            f->active   = true;
            f->timer_ms = 0U;
            return true;
        }
    } else if (!f->latching) {
        f->timer_ms = condition ? 0U : f->timer_ms + dt_ms;
        if (!condition && f->timer_ms >= f->clear_ms) {
            f->active   = false;
            f->timer_ms = 0U;
            return true;
        }
    }
    return false;
}

void fault_clear(fault_monitor_t *m, fault_id_t id)
{
    if (id < FAULT_COUNT) {
        m->item[id].active   = false;
        m->item[id].timer_ms = 0U;
    }
}

void fault_clear_all(fault_monitor_t *m)
{
    for (int i = 0; i < FAULT_COUNT; i++) {
        fault_clear(m, (fault_id_t)i);
    }
}

bool fault_active(const fault_monitor_t *m, fault_id_t id)
{
    return (id < FAULT_COUNT) && m->item[id].active;
}

uint32_t fault_bits(const fault_monitor_t *m)
{
    uint32_t bits = 0U;
    for (int i = 0; i < FAULT_COUNT; i++) {
        if (m->item[i].active) {
            bits |= 1UL << i;
        }
    }
    return bits;
}

const char *fault_name(fault_id_t id)
{
    static const char *const names[FAULT_COUNT] = {
        "FAN_STALL", "SENSOR", "OVER_TEMP", "STORAGE",
    };
    return (id < FAULT_COUNT) ? names[id] : "?";
}
