#include "display.h"
#include "ssd1306.h"
#include "lcd1602.h"
#include "sys_data.h"
#include "supervisor.h"
#include "fault.h"
#include "bsp.h"
#include "err.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* The display is found at run time: an SSD1306 OLED (0x3C) is preferred,
 * otherwise a 16x2 HD44780 LCD behind a PCF8574 (0x20..0x27) or PCF8574A
 * (0x38..0x3F) backpack. The usual defaults 0x27 / 0x3F are tried first.
 * Skipped: 0x38 (AHT20 sensor) and 0x3C / 0x3D (SSD1306 OLED). */
typedef enum { DISP_NONE = 0, DISP_OLED, DISP_LCD } disp_kind_t;

static disp_kind_t s_kind;
static ssd1306_t   s_oled;    /* 1 KB frame buffer, static on purpose */
static lcd1602_t   s_lcd;

void display_init(const i2c_bus_t *bus)
{
    static const uint8_t lcd_addr[] = {
        LCD1602_ADDR_DEFAULT, LCD1602_ADDR_ALT,
        0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U, 0x26U,
        0x39U, 0x3AU, 0x3BU, 0x3EU,
    };
    const char *name = "none";

    if (ssd1306_init(&s_oled, bus, SSD1306_ADDR) == ERR_OK) {
        s_kind = DISP_OLED;
        name = "SSD1306";
        log_printf("DISP", "SSD1306 OLED found at 0x%02X", SSD1306_ADDR);
    }
    for (unsigned i = 0U; s_kind == DISP_NONE && i < sizeof lcd_addr; i++) {
        if (lcd1602_init(&s_lcd, bus, lcd_addr[i], 16U, 2U, bsp_delay_ms) == ERR_OK) {
            s_kind = DISP_LCD;
            name = "LCD1602";
            log_printf("DISP", "LCD 16x2 (PCF8574) found at 0x%02X", lcd_addr[i]);
        }
    }
    if (s_kind == DISP_NONE) {
        log_printf("DISP", "no display (SSD1306 0x3C, LCD 0x20-0x27/0x39-0x3F)");
    }
    sys_lock();
    sys_data()->m.oled_ok      = (s_kind != DISP_NONE);
    sys_data()->m.display_name = name;
    sys_unlock();
}

static const char *first_fault(uint32_t faults)
{
    for (int i = 0; i < FAULT_COUNT; i++) {
        if (faults & (1UL << i)) {
            return fault_name((fault_id_t)i);
        }
    }
    return NULL;
}

static int draw_oled(const sys_data_t *d)
{
    char line[24], a[16], b[16];
    ssd1306_clear(&s_oled);
    snprintf(line, sizeof line, "FAN CTRL    %-6s", app_state_name(d->m.state));
    ssd1306_text(&s_oled, 0, 0, line);
    if (d->m.temp_valid) {
        snprintf(line, sizeof line, "T %sC H %s%%",
                 fmt_fixed(a, (int32_t)(d->m.temp_c * 10.0f), 1), fmt_fixed(b, (int32_t)(d->m.hum_pct * 10.0f), 1));
    } else {
        snprintf(line, sizeof line, "T --.-C  (sensor?)");
    }
    ssd1306_text(&s_oled, 0, 2, line);
    snprintf(line, sizeof line, "SP %sC  %u rpm", fmt_fixed(a, d->cfg.setpoint_c100 / 10, 1), d->m.rpm);
    ssd1306_text(&s_oled, 0, 3, line);
    snprintf(line, sizeof line, "Out %s%%", fmt_fixed(a, (int32_t)(d->m.duty * 10.0f), 1));
    ssd1306_text(&s_oled, 0, 4, line);
    ssd1306_bar(&s_oled, 0, 42, 128, 8, (int)(d->m.duty + 0.5f));
    const char *f = first_fault(d->m.faults);
    if (f != NULL) {
        snprintf(line, sizeof line, "! %s", f);
    } else {
        uint32_t up = bsp_millis() / 1000U;
        snprintf(line, sizeof line, "up %luh%02lum", (unsigned long)(up / 3600U), (unsigned long)((up / 60U) % 60U));
    }
    ssd1306_text(&s_oled, 0, 7, line);
    return ssd1306_flush(&s_oled);
}

/* 16x2:  "AUTO    T 30.4C"   /   "Out 45%  1350rpm"  (or the fault name,
 * shown every other second so the numbers stay readable). */
static int draw_lcd(const sys_data_t *d, uint32_t tick)
{
    char l0[24], l1[24], a[16];
    if (d->m.temp_valid) {
        snprintf(l0, sizeof l0, "%-7s T%sC", app_state_name(d->m.state), fmt_fixed(a, (int32_t)(d->m.temp_c * 10.0f), 1));
    } else {
        snprintf(l0, sizeof l0, "%-7s T --.-C", app_state_name(d->m.state));
    }
    const char *f = first_fault(d->m.faults);
    if (f != NULL && (tick / 2U) % 2U == 0U) {
        snprintf(l1, sizeof l1, "! %s", f);
    } else {
        snprintf(l1, sizeof l1, "Out%3u%% %5urpm", (unsigned)(d->m.duty + 0.5f), d->m.rpm);
    }
    int rc = lcd1602_line(&s_lcd, 0U, l0);
    return (rc == ERR_OK) ? lcd1602_line(&s_lcd, 1U, l1) : rc;
}

void display_task(void *arg)
{
    (void)arg;
    int sv = sv_register("display", 3000U);
    uint32_t tick = 0U;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        sv_kick(sv);
        tick++;
        if (s_kind == DISP_NONE) {
            continue;
        }
        sys_data_t d;
        sys_snapshot(&d);
        (void)((s_kind == DISP_OLED) ? draw_oled(&d) : draw_lcd(&d, tick));
    }
}
