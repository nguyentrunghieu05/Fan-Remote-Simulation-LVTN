/**
 * @file    display.h
 * @brief   Local status screen on the SSD1306 OLED (same I2C bus as SHT31).
 */
#ifndef DISPLAY_H
#define DISPLAY_H

#include "bus.h"

void display_init(const i2c_bus_t *bus);
void display_task(void *arg);

#endif /* DISPLAY_H */
