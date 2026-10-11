/**
 * @file    app_fsm.h
 * @brief   Operating-mode state machine of the fan controller.
 *
 *            BUTTON / MODE cmd
 *     +-----+  ------------>  +--------+  ------------>  +------+
 *     | OFF |                 | MANUAL |                 | AUTO |
 *     +-----+  <------------  +--------+  <------------  +------+
 *         \__________________ FAULT_RAISED ___________________/
 *                                  |
 *                              +-------+   FAULT_CLEARED: back to the
 *                              | FAULT |   mode that was active (or the
 *                              +-------+   last one requested meanwhile)
 *
 * Pure logic, no RTOS and no hardware: it is unit-tested on the PC.
 */
#ifndef APP_FSM_H
#define APP_FSM_H

#include <stdbool.h>

typedef enum {
    APP_STATE_OFF = 0,
    APP_STATE_MANUAL,
    APP_STATE_AUTO,
    APP_STATE_FAULT,
    APP_STATE_COUNT
} app_state_t;

typedef enum {
    APP_EV_BUTTON = 0,      /* short press on the user button: next mode */
    APP_EV_MODE_OFF,
    APP_EV_MODE_MANUAL,
    APP_EV_MODE_AUTO,
    APP_EV_FAULT_RAISED,
    APP_EV_FAULT_CLEARED,   /* no fault active any more */
} app_event_t;

typedef struct {
    app_state_t state;
    app_state_t resume;     /* mode to return to after a fault */
} app_fsm_t;

void        app_fsm_init(app_fsm_t *f, app_state_t initial);
/** @return true when the state changed. */
bool        app_fsm_handle(app_fsm_t *f, app_event_t ev);
const char *app_state_name(app_state_t s);

#endif /* APP_FSM_H */
