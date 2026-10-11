#include "lcd1602.h"
#include "err.h"

#define PIN_RS   0x01U
#define PIN_EN   0x04U
#define PIN_BL   0x08U

#define CMD_CLEAR        0x01U
#define CMD_ENTRY_INC    0x06U
#define CMD_DISPLAY_ON   0x0CU      /* display on, cursor off, blink off */
#define CMD_FUNC_4BIT_2L 0x28U      /* 4-bit bus, 2 (or 4) lines, 5x8 font */
#define CMD_DDRAM        0x80U

/* Append one nibble (upper 4 bits of 'nib') as an EN pulse: 2 port values. */
static size_t put_nibble(const lcd1602_t *d, uint8_t *buf, size_t n, uint8_t nib, uint8_t rs)
{
    uint8_t v = (uint8_t)((nib & 0xF0U) | d->backlight | rs);
    buf[n++] = (uint8_t)(v | PIN_EN);
    buf[n++] = v;                              /* falling edge latches the data */
    return n;
}

static size_t put_byte(const lcd1602_t *d, uint8_t *buf, size_t n, uint8_t b, uint8_t rs)
{
    n = put_nibble(d, buf, n, b, rs);
    return put_nibble(d, buf, n, (uint8_t)(b << 4), rs);
}

static int send(lcd1602_t *d, uint8_t b, uint8_t rs)
{
    uint8_t buf[4];
    size_t n = put_byte(d, buf, 0U, b, rs);
    return d->bus.write(d->bus.ctx, d->addr, buf, n);
}

int lcd1602_init(lcd1602_t *d, const i2c_bus_t *bus, uint8_t addr, uint8_t cols, uint8_t rows,
                 delay_ms_fn delay_ms)
{
    d->bus       = *bus;
    d->addr      = addr;
    d->cols      = (cols > LCD_MAX_COLS) ? LCD_MAX_COLS : cols;
    d->rows      = (rows > 4U) ? 4U : rows;
    d->delay_ms  = delay_ms;
    d->backlight = PIN_BL;

    uint8_t idle = 0U;
    int rc = d->bus.write(d->bus.ctx, d->addr, &idle, 1U);   /* NACK = no backpack */
    if (rc != ERR_OK) {
        return rc;
    }
    d->delay_ms(50U);                          /* LCD power-on time */

    /* HD44780 datasheet "initialization by instruction" for 4-bit mode:
     * the controller may be in 8-bit or in the middle of a 4-bit transfer,
     * so send 0x3 three times, then 0x2 to switch to 4 bits. */
    static const uint8_t wake[] = { 0x30U, 0x30U, 0x30U, 0x20U };
    static const uint8_t wait_ms[] = { 5U, 1U, 1U, 1U };
    for (unsigned i = 0U; rc == ERR_OK && i < sizeof wake; i++) {
        uint8_t buf[2];
        size_t n = put_nibble(d, buf, 0U, wake[i], 0U);
        rc = d->bus.write(d->bus.ctx, d->addr, buf, n);
        d->delay_ms(wait_ms[i]);
    }
    if (rc == ERR_OK) { rc = send(d, CMD_FUNC_4BIT_2L, 0U); }
    if (rc == ERR_OK) { rc = send(d, CMD_DISPLAY_ON, 0U); }
    if (rc == ERR_OK) { rc = send(d, CMD_ENTRY_INC, 0U); }
    if (rc == ERR_OK) { rc = lcd1602_clear(d); }
    return rc;
}

int lcd1602_clear(lcd1602_t *d)
{
    int rc = send(d, CMD_CLEAR, 0U);
    d->delay_ms(2U);                           /* clear takes 1.52 ms */
    return rc;
}

int lcd1602_line(lcd1602_t *d, uint8_t row, const char *s)
{
    static const uint8_t row_addr[4] = { 0x00U, 0x40U, 0x14U, 0x54U };
    if (row >= d->rows) {
        return ERR_PARAM;
    }
    /* address command + one data byte per column, 4 port values each */
    uint8_t buf[4U * (1U + LCD_MAX_COLS)];
    size_t n = put_byte(d, buf, 0U, (uint8_t)(CMD_DDRAM | row_addr[row]), 0U);
    bool end = false;
    for (uint8_t c = 0U; c < d->cols; c++) {
        char ch = end ? ' ' : s[c];
        if (ch == '\0') {
            end = true;
            ch = ' ';
        }
        n = put_byte(d, buf, n, (uint8_t)ch, PIN_RS);
    }
    /* Even at 400 kHz each port value takes ~23 us, so every byte is spaced
     * >= 90 us apart: longer than the 37 us an HD44780 needs per write.
     * The bus runs at 100 kHz (board.h), which only adds margin. */
    return d->bus.write(d->bus.ctx, d->addr, buf, n);
}

int lcd1602_backlight(lcd1602_t *d, bool on)
{
    d->backlight = on ? PIN_BL : 0U;
    uint8_t v = d->backlight;
    return d->bus.write(d->bus.ctx, d->addr, &v, 1U);
}
