#include "display.h"
#include "ssd1306.h"
#include "sys_data.h"
#include "supervisor.h"
#include "fault.h"
#include "bsp.h"
#include "err.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

static ssd1306_t s_oled;    /* 1 KB frame buffer, static on purpose */
static bool      s_ok;

void display_init(const i2c_bus_t *bus)
{
    s_ok = (ssd1306_init(&s_oled, bus, SSD1306_ADDR) == ERR_OK);
    log_printf("OLED", "SSD1306 %s", s_ok ? "found" : "NOT found");
    sys_lock();
    sys_data()->m.oled_ok = s_ok;
    sys_unlock();
}

void display_task(void *arg)
{
    (void)arg;
    int sv = sv_register("display", 3000U);
    char line[24], a[16], b[16];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        sv_kick(sv);
        if (!s_ok) {
            continue;
        }
        sys_data_t d;
        sys_snapshot(&d);
        ssd1306_clear(&s_oled);

        snprintf(line, sizeof line, "FAN CTRL    %-6s", app_state_name(d.m.state));
        ssd1306_text(&s_oled, 0, 0, line);
        if (d.m.temp_valid) {
            snprintf(line, sizeof line, "T %sC H %s%%",
                     fmt_fixed(a, (int32_t)(d.m.temp_c * 10.0f), 1), fmt_fixed(b, (int32_t)(d.m.hum_pct * 10.0f), 1));
        } else {
            snprintf(line, sizeof line, "T --.-C  (sensor?)");
        }
        ssd1306_text(&s_oled, 0, 2, line);
        snprintf(line, sizeof line, "SP %sC  %u rpm", fmt_fixed(a, d.cfg.setpoint_c100 / 10, 1), d.m.rpm);
        ssd1306_text(&s_oled, 0, 3, line);
        snprintf(line, sizeof line, "Out %s%%", fmt_fixed(a, (int32_t)(d.m.duty * 10.0f), 1));
        ssd1306_text(&s_oled, 0, 4, line);
        ssd1306_bar(&s_oled, 0, 42, 128, 8, (int)(d.m.duty + 0.5f));
        if (d.m.faults != 0U) {
            for (int i = 0; i < FAULT_COUNT; i++) {
                if (d.m.faults & (1UL << i)) {
                    snprintf(line, sizeof line, "! %s", fault_name((fault_id_t)i));
                    break;
                }
            }
        } else {
            uint32_t up = bsp_millis() / 1000U;
            snprintf(line, sizeof line, "up %luh%02lum", (unsigned long)(up / 3600U), (unsigned long)((up / 60U) % 60U));
        }
        ssd1306_text(&s_oled, 0, 7, line);
        (void)ssd1306_flush(&s_oled);
    }
}
