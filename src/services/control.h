/**
 * @file    control.h
 * @brief   Control task: state machine, faults, fan output, alarm relay.
 */
#ifndef CONTROL_H
#define CONTROL_H

#define CONTROL_PERIOD_MS  100U

void control_init(void);
void control_task(void *arg);

#endif /* CONTROL_H */
