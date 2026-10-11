/**
 * @file    ssd1306.h
 * @brief   SSD1306 128x64 monochrome OLED over I2C (address 0x3C).
 *
 * Shares the I2C bus with the SHT3x: two devices, two addresses, one bus.
 * Drawing happens in a RAM frame buffer (1 KB); ssd1306_flush() sends it
 * to the panel in 8 page writes.
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

#define SSD1306_ADDR     0x3CU
#define SSD1306_WIDTH    128U
#define SSD1306_HEIGHT   64U

typedef struct {
    i2c_bus_t bus;
    uint8_t   addr;
    uint8_t   fb[SSD1306_WIDTH * SSD1306_HEIGHT / 8U];
} ssd1306_t;

int  ssd1306_init(ssd1306_t *d, const i2c_bus_t *bus, uint8_t addr);
void ssd1306_clear(ssd1306_t *d);
void ssd1306_pixel(ssd1306_t *d, int x, int y, bool on);
/** Draw text with the built-in 5x7 font; line = 0..7 (8 px rows). */
void ssd1306_text(ssd1306_t *d, int x, int line, const char *s);
/** Horizontal bar, used as a fan speed gauge. percent 0..100. */
void ssd1306_bar(ssd1306_t *d, int x, int y, int w, int h, int percent);
int  ssd1306_flush(ssd1306_t *d);

#endif /* SSD1306_H */
