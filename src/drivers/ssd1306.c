#include "ssd1306.h"
#include "err.h"
#include <string.h>

#include "font5x7.inc"

#define CTRL_CMD   0x00U    /* control byte: following bytes are commands */
#define CTRL_DATA  0x40U    /* control byte: following bytes are GDDRAM data */

static int cmds(ssd1306_t *d, const uint8_t *c, size_t n)
{
    uint8_t buf[32];
    if (n + 1U > sizeof buf) {
        return ERR_PARAM;
    }
    buf[0] = CTRL_CMD;
    memcpy(&buf[1], c, n);
    return d->bus.write(d->bus.ctx, d->addr, buf, n + 1U);
}

int ssd1306_init(ssd1306_t *d, const i2c_bus_t *bus, uint8_t addr)
{
    static const uint8_t seq[] = {
        0xAE,             /* display off                      */
        0xD5, 0x80,       /* clock divide                     */
        0xA8, 0x3F,       /* multiplex 64                     */
        0xD3, 0x00,       /* display offset 0                 */
        0x40,             /* start line 0                     */
        0x8D, 0x14,       /* charge pump on (internal VCC)    */
        0x20, 0x00,       /* horizontal addressing mode       */
        0xA1, 0xC8,       /* segment remap + COM scan reversed (normal orientation) */
        0xDA, 0x12,       /* COM pins config for 128x64       */
        0x81, 0x7F,       /* contrast                         */
        0xD9, 0xF1,       /* pre-charge                       */
        0xDB, 0x40,       /* VCOMH                            */
        0xA4, 0xA6,       /* resume RAM content, normal (not inverted) */
        0xAF,             /* display on                       */
    };
    d->bus  = *bus;
    d->addr = addr;
    ssd1306_clear(d);
    return cmds(d, seq, sizeof seq);
}

void ssd1306_clear(ssd1306_t *d)
{
    memset(d->fb, 0, sizeof d->fb);
}

void ssd1306_pixel(ssd1306_t *d, int x, int y, bool on)
{
    if (x < 0 || y < 0 || x >= (int)SSD1306_WIDTH || y >= (int)SSD1306_HEIGHT) {
        return;
    }
    uint8_t *byte = &d->fb[(y / 8) * SSD1306_WIDTH + x];
    uint8_t  mask = (uint8_t)(1U << (y % 8));
    *byte = on ? (uint8_t)(*byte | mask) : (uint8_t)(*byte & ~mask);
}

void ssd1306_text(ssd1306_t *d, int x, int line, const char *s)
{
    if (line < 0 || line > 7) {
        return;
    }
    for (; *s != '\0' && x <= (int)SSD1306_WIDTH - 6; s++, x += 6) {
        char c = (*s < 0x20 || *s > 0x7E) ? '?' : *s;
        memcpy(&d->fb[line * SSD1306_WIDTH + x], font5x7[c - 0x20], 5U);
        d->fb[line * SSD1306_WIDTH + x + 5] = 0U;          /* 1 px spacing */
    }
}

void ssd1306_bar(ssd1306_t *d, int x, int y, int w, int h, int percent)
{
    if (percent < 0)   { percent = 0; }
    if (percent > 100) { percent = 100; }
    int fill = (w - 2) * percent / 100;
    for (int i = 0; i < w; i++) {
        ssd1306_pixel(d, x + i, y, true);
        ssd1306_pixel(d, x + i, y + h - 1, true);
    }
    for (int j = 0; j < h; j++) {
        ssd1306_pixel(d, x, y + j, true);
        ssd1306_pixel(d, x + w - 1, y + j, true);
    }
    for (int i = 0; i < fill; i++) {
        for (int j = 2; j < h - 2; j++) {
            ssd1306_pixel(d, x + 1 + i, y + j, true);
        }
    }
}

int ssd1306_flush(ssd1306_t *d)
{
    static const uint8_t window[] = { 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07 };
    int rc = cmds(d, window, sizeof window);
    uint8_t buf[1U + SSD1306_WIDTH];
    for (unsigned page = 0U; rc == ERR_OK && page < 8U; page++) {
        buf[0] = CTRL_DATA;
        memcpy(&buf[1], &d->fb[page * SSD1306_WIDTH], SSD1306_WIDTH);
        rc = d->bus.write(d->bus.ctx, d->addr, buf, sizeof buf);
    }
    return rc;
}
