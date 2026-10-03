/**
 * @file    sensors.h
 * @brief   Sensor task: SHT31 (I2C), speed knob (ADC), tach, user button.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include "bus.h"

/** @param bus the I2C bus shared with the display. */
void sensors_init(const i2c_bus_t *bus);
void sensors_task(void *arg);

#endif /* SENSORS_H */
