/**
 * @file    lcd1602.h
 * @brief   HD44780 character LCD (16x2 / 20x4) behind a PCF8574 I2C backpack.
 *
 * The PCF8574 is an 8-bit I2C port expander. On the common backpacks:
 *   P0 = RS, P1 = RW, P2 = EN, P3 = backlight, P4..P7 = LCD D4..D7
 * so the LCD runs in 4-bit mode: every byte is sent as two nibbles, each
 * latched by a high -> low pulse on EN. One I2C write carries several port
 * values in a row, so a whole text line goes out in a single transaction.
 *
 * Note: most of these LCDs need 5 V for good contrast; at 3.3 V the
 * PCF8574 still answers but the text can be faint.
 */
#ifndef LCD1602_H
#define LCD1602_H

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

#define LCD1602_ADDR_DEFAULT  0x27U   /* PCF8574,  A2..A0 open */
#define LCD1602_ADDR_ALT      0x3FU   /* PCF8574A, A2..A0 open */
#define LCD_MAX_COLS          20U

typedef struct {
    i2c_bus_t   bus;
    delay_ms_fn delay_ms;
    uint8_t     addr;
    uint8_t     cols, rows;
    uint8_t     backlight;      /* 0 or the P3 bit */
} lcd1602_t;

int  lcd1602_init(lcd1602_t *d, const i2c_bus_t *bus, uint8_t addr, uint8_t cols, uint8_t rows,
                  delay_ms_fn delay_ms);
int  lcd1602_clear(lcd1602_t *d);
/** Write text at (0, row), padded with spaces to the full width (no flicker, no clear needed). */
int  lcd1602_line(lcd1602_t *d, uint8_t row, const char *s);
int  lcd1602_backlight(lcd1602_t *d, bool on);

#endif /* LCD1602_H */
