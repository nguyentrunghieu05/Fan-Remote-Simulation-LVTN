#include "app_fsm.h"

void app_fsm_init(app_fsm_t *f, app_state_t initial)
{
    f->state  = (initial == APP_STATE_FAULT || initial >= APP_STATE_COUNT) ? APP_STATE_OFF : initial;
    f->resume = f->state;
}

static app_state_t mode_of(app_event_t ev, app_state_t cur)
{
    switch (ev) {
    case APP_EV_MODE_OFF:    return APP_STATE_OFF;
    case APP_EV_MODE_MANUAL: return APP_STATE_MANUAL;
    case APP_EV_MODE_AUTO:   return APP_STATE_AUTO;
    case APP_EV_BUTTON:      return (cur == APP_STATE_OFF)    ? APP_STATE_MANUAL
                                  : (cur == APP_STATE_MANUAL) ? APP_STATE_AUTO
                                  :                             APP_STATE_OFF;
    default:                 return cur;
    }
}

bool app_fsm_handle(app_fsm_t *f, app_event_t ev)
{
    app_state_t old = f->state;

    if (f->state == APP_STATE_FAULT) {
        if (ev == APP_EV_FAULT_CLEARED) {
            f->state = f->resume;
        } else if (ev != APP_EV_FAULT_RAISED) {
            /* A mode request during a fault is remembered, not applied. */
            f->resume = mode_of(ev, f->resume);
        }
    } else if (ev == APP_EV_FAULT_RAISED) {
        f->resume = f->state;
        f->state  = APP_STATE_FAULT;
    } else if (ev != APP_EV_FAULT_CLEARED) {
        f->state  = mode_of(ev, f->state);
        f->resume = f->state;
    }
    return f->state != old;
}

const char *app_state_name(app_state_t s)
{
    static const char *const n[APP_STATE_COUNT] = { "OFF", "MANUAL", "AUTO", "FAULT" };
    return (s < APP_STATE_COUNT) ? n[s] : "?";
}
