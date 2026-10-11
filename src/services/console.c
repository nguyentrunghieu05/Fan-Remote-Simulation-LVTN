#include "console.h"
#include "cli.h"
#include "log.h"
#include "bsp.h"
#include "board.h"
#include "err.h"
#include "sys_data.h"
#include "storage.h"
#include "fault.h"
#include "timeconv.h"
#include "modbus_server.h"
#include "radio_link.h"
#include "supervisor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static cli_t        s_cli;
static TaskHandle_t s_task;
static int          s_sv = -1;

static void on_rx(bsp_uart_t port)
{
    (void)port;
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* Parse "27.5" -> 2750 (x100) without strtof (keeps code small). */
static bool parse_x100(const char *s, int32_t *out)
{
    char *end;
    long ip = strtol(s, &end, 10);
    long fp = 0;
    int  digits = 0;
    if (*end == '.') {
        end++;
        while (*end >= '0' && *end <= '9' && digits < 2) {
            fp = fp * 10 + (*end++ - '0');
            digits++;
        }
        if (digits == 1) {
            fp *= 10;
        }
    }
    if (*end != '\0') {
        return false;
    }
    *out = (int32_t)(ip * 100 + ((s[0] == '-') ? -fp : fp));
    return true;
}

/* ------------------------------------------------------------- commands */
static int cmd_status(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    sys_data_t d;
    char a[16], b[16];
    sys_snapshot(&d);
    cli_printf(c, "state     %s\r\n", app_state_name(d.m.state));
    if (d.m.temp_valid) {
        cli_printf(c, "temp      %s C   hum %s %%", fmt_fixed(a, (int32_t)(d.m.temp_c * 100.0f), 2),
                   fmt_fixed(b, (int32_t)(d.m.hum_pct * 10.0f), 1));
        if (d.m.press_hpa > 0.0f) {
            cli_printf(c, "   press %s hPa", fmt_fixed(a, (int32_t)(d.m.press_hpa * 10.0f), 1));
        }
        cli_printf(c, "   (%s)\r\n", d.m.sensor_name);
    } else {
        cli_printf(c, "temp      no sensor (%lu errors)\r\n", (unsigned long)d.m.sensor_errors);
    }
    cli_printf(c, "setpoint  %s C\r\n", fmt_fixed(a, d.cfg.setpoint_c100, 2));
    cli_printf(c, "output    %s %%   fan %u rpm   knob %s %%\r\n", fmt_fixed(a, (int32_t)(d.m.duty * 10.0f), 1),
               d.m.rpm, fmt_fixed(b, (int32_t)(d.m.pot_percent * 10.0f), 1));
    cli_printf(c, "faults    0x%02lX", (unsigned long)d.m.faults);
    for (int i = 0; i < FAULT_COUNT; i++) {
        if (d.m.faults & (1UL << i)) {
            cli_printf(c, " %s", fault_name((fault_id_t)i));
        }
    }
    cli_printf(c, "\r\nflash     %s   display %s   uptime %lu s\r\n", d.m.flash_ok ? "ok" : "missing",
               d.m.display_name, (unsigned long)(bsp_millis() / 1000U));
    return 0;
}

static int cmd_mode(cli_t *c, int argc, char *argv[])
{
    static const struct { const char *n; app_event_t e; } m[] = {
        { "off", APP_EV_MODE_OFF }, { "manual", APP_EV_MODE_MANUAL }, { "auto", APP_EV_MODE_AUTO },
    };
    for (size_t i = 0; argc == 2 && i < 3; i++) {
        if (strcmp(argv[1], m[i].n) == 0) {
            (void)app_post_event(m[i].e);
            return 0;
        }
    }
    cli_printf(c, "usage: mode off|manual|auto\r\n");
    return 1;
}

static int cmd_duty(cli_t *c, int argc, char *argv[])
{
    int32_t v;
    if (argc == 2 && strcmp(argv[1], "knob") == 0) {
        sys_lock(); sys_data()->cfg.use_pot = 1U; sys_unlock();
        return 0;
    }
    if (argc != 2 || !parse_x100(argv[1], &v) || v < 0 || v > 10000) {
        cli_printf(c, "usage: duty <0..100> | duty knob\r\n");
        return 1;
    }
    sys_lock();
    sys_data()->cfg.manual_duty_c10 = (uint16_t)(v / 10);
    sys_data()->cfg.use_pot = 0U;
    sys_unlock();
    return 0;
}

static int cmd_setpoint(cli_t *c, int argc, char *argv[])
{
    int32_t v;
    if (argc != 2 || !parse_x100(argv[1], &v) || v < 0 || v > 8000) {
        cli_printf(c, "usage: setpoint <0..80 C>\r\n");
        return 1;
    }
    sys_lock(); sys_data()->cfg.setpoint_c100 = (int16_t)v; sys_unlock();
    return 0;
}

static int cmd_pid(cli_t *c, int argc, char *argv[])
{
    int32_t kp, ki, kd;
    char a[16], b[16], e[16];
    if (argc == 4 && parse_x100(argv[1], &kp) && parse_x100(argv[2], &ki) && parse_x100(argv[3], &kd)) {
        sys_lock();
        sys_data()->cfg.kp = kp / 100.0f;
        sys_data()->cfg.ki = ki / 100.0f;
        sys_data()->cfg.kd = kd / 100.0f;
        (void)fan_config_sanitize(&sys_data()->cfg);
        sys_unlock();
    } else if (argc != 1) {
        cli_printf(c, "usage: pid [kp ki kd]\r\n");
        return 1;
    }
    sys_data_t d;
    sys_snapshot(&d);
    cli_printf(c, "kp %s  ki %s  kd %s\r\n", fmt_fixed(a, (int32_t)(d.cfg.kp * 100.0f), 2),
               fmt_fixed(b, (int32_t)(d.cfg.ki * 100.0f), 2), fmt_fixed(e, (int32_t)(d.cfg.kd * 100.0f), 2));
    return 0;
}

static int cmd_cfg(cli_t *c, int argc, char *argv[])
{
    sys_data_t d;
    if (argc == 2 && strcmp(argv[1], "save") == 0) {
        sys_snapshot(&d);
        int rc = storage_save_config(&d.cfg);
        cli_printf(c, rc == ERR_OK ? "saved\r\n" : "save failed (%d)\r\n", rc);
        return rc;
    }
    if (argc == 2 && strcmp(argv[1], "default") == 0) {
        sys_lock(); fan_config_defaults(&sys_data()->cfg); sys_unlock();
        cli_printf(c, "defaults loaded (not saved)\r\n");
        return 0;
    }
    if (argc == 4 && strcmp(argv[1], "set") == 0) {
        long v = strtol(argv[3], NULL, 10);
        sys_lock();
        fan_config_t *k = &sys_data()->cfg;
        if      (strcmp(argv[2], "addr") == 0)     { k->modbus_addr = (uint8_t)v; }
        else if (strcmp(argv[2], "start") == 0)    { k->start_mode = (uint8_t)v; }
        else if (strcmp(argv[2], "mindut") == 0)   { k->min_duty_c10 = (uint16_t)v; }
        else if (strcmp(argv[2], "faultdut") == 0) { k->fault_duty_c10 = (uint16_t)v; }
        else if (strcmp(argv[2], "overtemp") == 0) { k->over_temp_c100 = (int16_t)v; }
        else if (strcmp(argv[2], "ramp") == 0)     { k->ramp_c10_per_s = (uint16_t)v; }
        else if (strcmp(argv[2], "stall") == 0)    { k->stall_rpm = (uint16_t)v; }
        else if (strcmp(argv[2], "logper") == 0)   { k->log_period_s = (uint16_t)v; }
        else if (strcmp(argv[2], "tlmper") == 0)   { k->tlm_period_s = (uint16_t)v; }
        else if (strcmp(argv[2], "node") == 0)     { k->node_id = (uint16_t)v; }
        else { sys_unlock(); cli_printf(c, "unknown key\r\n"); return 1; }
        (void)fan_config_sanitize(k);
        sys_unlock();
    }
    sys_snapshot(&d);
    const fan_config_t *k = &d.cfg;
    cli_printf(c, "addr %u  start %u  node %u\r\n", k->modbus_addr, k->start_mode, k->node_id);
    cli_printf(c, "mindut %u  faultdut %u  (0.1 %%)  ramp %u (0.1 %%/s)\r\n",
               k->min_duty_c10, k->fault_duty_c10, k->ramp_c10_per_s);
    cli_printf(c, "overtemp %d (0.01 C)  stall %u rpm\r\n", k->over_temp_c100, k->stall_rpm);
    cli_printf(c, "logper %u s  tlmper %u s\r\n", k->log_period_s, k->tlm_period_s);
    cli_printf(c, "keys: cfg set <key> <int> | cfg save | cfg default\r\n");
    return 0;
}

static int cmd_log(cli_t *c, int argc, char *argv[])
{
    if (argc >= 2 && strcmp(argv[1], "clear") == 0) {
        cli_printf(c, "erasing...\r\n");
        sv_maintenance(true);
        int rc = storage_log_format();
        sv_maintenance(false);
        cli_printf(c, "log %s\r\n", rc == ERR_OK ? "erased" : "erase failed");
        return rc;
    }
    uint32_t n = storage_log_count();
    cli_printf(c, "%lu / %lu records\r\n", (unsigned long)n, (unsigned long)storage_log_capacity());
    if (argc >= 2 && strcmp(argv[1], "dump") == 0) {
        uint32_t want = (argc >= 3) ? (uint32_t)strtoul(argv[2], NULL, 10) : 10U;
        uint32_t from = (want < n) ? n - want : 0U;
        cli_printf(c, "seq,unix,temp_c100,hum_c100,rpm,duty_c10,state,faults\r\n");   /* CSV */
        for (uint32_t i = from; i < n; i++) {
            datalog_sample_t s;
            uint32_t seq;
            sv_kick(s_sv);
            if (storage_log_read(i, &s, &seq) == ERR_OK) {
                cli_printf(c, "%lu,%lu,%d,%u,%u,%u,%u,%u\r\n", (unsigned long)seq, (unsigned long)s.timestamp,
                           s.temp_c100, s.hum_c100, s.rpm, s.duty_c10, s.state, s.faults);
            }
        }
    }
    return 0;
}

static int cmd_flash(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    if (!storage_ok()) {
        cli_printf(c, "no SPI flash\r\n");
        return 1;
    }
    cli_printf(c, "JEDEC id %06lX, size %lu KB\r\n", (unsigned long)storage_jedec_id(),
               (unsigned long)(storage_size() / 1024U));
    return 0;
}

/* What usually sits at this address (a scan cannot know for sure). */
static const char *i2c_guess(uint8_t a)
{
    switch (a) {
    case 0x20: case 0x21: case 0x22: case 0x23:
    case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x39: case 0x3A: case 0x3B:
    case 0x3E: case 0x3F:          return "PCF8574 (LCD backpack)";
    case 0x29:                     return "VL53L0X";
    case 0x38:                     return "AHT10/AHT20";
    case 0x3C: case 0x3D:          return "SSD1306 OLED";
    case 0x44: case 0x45:          return "SHT3x";
    case 0x76: case 0x77:          return "BME280/BMP280";
    default:                       return "";
    }
}

static int cmd_i2c(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (bsp_i2c_probe(a)) {
            cli_printf(c, "  0x%02X  %s\r\n", a, i2c_guess(a));
            found++;
        }
    }
    cli_printf(c, "%d device(s)\r\n", found);
    return 0;
}

static int cmd_time(cli_t *c, int argc, char *argv[])
{
    if (argc == 3 && strcmp(argv[1], "set") == 0) {
        bsp_rtc_set_unix((uint32_t)strtoul(argv[2], NULL, 10));
    }
    datetime_t dt;
    uint32_t t = bsp_rtc_get_unix();
    unix_to_datetime(t, &dt);
    cli_printf(c, "%04u-%02u-%02u %02u:%02u:%02u UTC (%lu)%s\r\n", dt.year, dt.month, dt.day, dt.hour,
               dt.minute, dt.second, (unsigned long)t, bsp_rtc_is_set() ? "" : "  not set: time set <unix>");
    return 0;
}

static int cmd_fault(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    app_request_fault_reset();
    cli_printf(c, "fault reset requested\r\n");
    return 0;
}

static int cmd_tasks(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    static char buf[640];
    vTaskList(buf);
    cli_printf(c, "name         state prio stack_free num\r\n");
    console_write(buf);
    vTaskGetRunTimeStats(buf);
    cli_printf(c, "name         time_us      cpu\r\n");
    console_write(buf);
    cli_printf(c, "heap free %u, min ever %u bytes\r\n", (unsigned)xPortGetFreeHeapSize(),
               (unsigned)xPortGetMinimumEverFreeHeapSize());
    return 0;
}

static int cmd_stats(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    uint32_t f, e, x, s, a, k;
    modbus_server_stats(&f, &e, &x);
    radio_link_stats(&s, &a, &k);
    cli_printf(c, "modbus  frames %lu  crc_err %lu  exceptions %lu  uart_err %lu\r\n", (unsigned long)f,
               (unsigned long)e, (unsigned long)x, (unsigned long)bsp_uart_errors(BSP_UART_RS485));
    cli_printf(c, "lora    sent %lu  acked %lu  cmds %lu\r\n", (unsigned long)s, (unsigned long)a, (unsigned long)k);
    return 0;
}

static int cmd_reboot(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    cli_printf(c, "rebooting...\r\n");
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_reset();
    return 0;
}

static const cli_cmd_t s_cmds[] = {
    { "status",   "show measurements and state",            cmd_status },
    { "mode",     "off | manual | auto",                    cmd_mode },
    { "duty",     "<0..100> manual output, or 'knob'",      cmd_duty },
    { "setpoint", "<C> auto-mode temperature",              cmd_setpoint },
    { "pid",      "[kp ki kd] show / set PID gains",        cmd_pid },
    { "cfg",      "show | set <key> <v> | save | default",  cmd_cfg },
    { "log",      "count | dump [n] (CSV) | clear",         cmd_log },
    { "flash",    "SPI flash info",                         cmd_flash },
    { "i2c",      "scan the I2C bus",                       cmd_i2c },
    { "time",     "show | set <unix>",                      cmd_time },
    { "fault",    "clear latched faults",                   cmd_fault },
    { "tasks",    "RTOS tasks, stack and CPU usage",        cmd_tasks },
    { "stats",    "Modbus / LoRa counters",                 cmd_stats },
    { "reboot",   "software reset",                         cmd_reboot },
};

void console_init(void)
{
    bsp_uart_init(BSP_UART_CONSOLE, CONSOLE_BAUD, on_rx);
    cli_init(&s_cli, s_cmds, sizeof s_cmds / sizeof s_cmds[0], console_write, true);
}

void console_task(void *arg)
{
    (void)arg;
    s_task = xTaskGetCurrentTaskHandle();
    s_sv = sv_register("console", 3000U);
    console_write("\r\ntype 'help'\r\n");
    console_write(s_cli.prompt);
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        sv_kick(s_sv);
        char ch;
        while (bsp_uart_read(BSP_UART_CONSOLE, &ch, 1U) == 1U) {
            cli_input(&s_cli, ch);
        }
    }
}
