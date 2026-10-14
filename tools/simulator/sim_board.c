/**
 * @file    sim_board.c
 * @brief   Virtual board for the PC / browser simulator (software-in-the-loop).
 *
 * The firmware's portable layers are compiled UNCHANGED next to this file:
 *   app_fsm, fan_logic, pid, ramp, fault, modbus_slave, crc, telemetry,
 *   datalog, cli, w25qxx, sht3x, ssd1306.
 *
 * What is simulated here instead of the real hardware:
 *   - the room / machine (thermal model) and the fan (speed, inertia, tach)
 *   - the chips on the buses, at the byte level: W25Q64 on SPI, SHT31 and
 *     SSD1306 on I2C - so the real drivers run against them
 *   - the RTOS tasks: one 100 ms tick calls the same steps in the same
 *     order as control / sensors / storage / display / link tasks do.
 *
 * Built with clang/zig for wasm32 (browser) or with gcc for a quick native
 * test (sim_main.c). Every exported function starts with sim_.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "err.h"
#include "bus.h"
#include "app_fsm.h"
#include "fan_logic.h"
#include "fault.h"
#include "modbus_slave.h"
#include "datalog.h"
#include "telemetry.h"
#include "cli.h"
#include "crc.h"
#include "w25qxx.h"
#include "sht3x.h"
#include "ssd1306.h"

#ifdef __wasm__
#define EXPORT(name) __attribute__((export_name(#name)))
#else
#define EXPORT(name)
#endif

#define TICK_MS         100U
#define CFG_LOG_BASE    0x10000UL
#define RPM_MAX         3000.0f
#define FW_VERSION_REG  0x0100U
#define CRITICAL_FAULTS ((1UL << FAULT_FAN_STALL) | (1UL << FAULT_SENSOR) | (1UL << FAULT_OVER_TEMP))

/* ======================================================================
 *  Text outputs read by the UI (console, LoRa air, Modbus reply)
 * ====================================================================== */
#define OUT_CAP 16384
typedef struct { char buf[OUT_CAP]; size_t len; } outbuf_t;
static outbuf_t s_console, s_air;

static void out_put(outbuf_t *o, const char *s)
{
    size_t n = strlen(s);
    if (o->len + n >= OUT_CAP) {            /* UI did not drain: drop oldest */
        o->len = 0;
    }
    memcpy(&o->buf[o->len], s, n);
    o->len += n;
    o->buf[o->len] = '\0';
}

EXPORT(sim_console_out) const char *sim_console_out(void) { return s_console.buf; }
EXPORT(sim_air_out)     const char *sim_air_out(void)     { return s_air.buf; }
EXPORT(sim_outputs_clear) void sim_outputs_clear(void)
{
    s_console.len = 0; s_console.buf[0] = '\0';
    s_air.len = 0;     s_air.buf[0] = '\0';
}

static uint32_t s_ms;          /* time since reset */
static uint32_t s_unix0;       /* RTC value at reset */

static void log_printf(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
#include <stdarg.h>
static void log_printf(const char *tag, const char *fmt, ...)
{
    char msg[128], line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    snprintf(line, sizeof line, "[%6lu.%01lu] %-5s %s\r\n",
             (unsigned long)(s_ms / 1000U), (unsigned long)((s_ms / 100U) % 10U), tag, msg);
    out_put(&s_console, line);
}

static void no_delay(uint32_t ms) { (void)ms; }

/* ======================================================================
 *  Plant: machine heat, room, fan
 * ====================================================================== */
typedef struct {
    float ambient_c;      /* room temperature                          */
    float heat_pct;       /* heat produced by the machine, 0..100 %    */
    float temp_c;         /* air temperature at the sensor             */
    float hum_pct;
    float rpm;            /* real fan speed                            */
    bool  fan_blocked;    /* injected fault: rotor jammed              */
    bool  sht_unplugged;  /* injected fault: sensor cable off          */
    bool  oled_unplugged;
    bool  flash_missing;
    bool  gateway_online; /* LoRa gateway answers with $ACK            */
    float pot_pct;        /* knob position                             */
    float duty_out;       /* what the MCU drives (PWM / DAC)           */
} plant_t;

static plant_t P;

static uint32_t s_rng = 12345U;
static float noise(float amp)
{
    s_rng = s_rng * 1103515245U + 12345U;
    return amp * (((s_rng >> 9) & 0xFFFF) / 32768.0f - 1.0f);
}

/* Thermal model:  tau * dT/dt = H - (g0 + g1 * airflow) * (T - Ta)
 * H = 0.4 °C per % of heat: 100 % heat with the fan stopped = +40 °C;
 * full fan airflow divides the temperature rise by 5. */
static void plant_step(float dt)
{
    const float tau = 45.0f, g0 = 1.0f, g1 = 4.0f;
    float target_rpm = (P.duty_out >= 8.0f && !P.fan_blocked) ? RPM_MAX * P.duty_out / 100.0f : 0.0f;
    P.rpm += (target_rpm - P.rpm) * (dt / 1.2f);            /* rotor inertia */
    if (P.fan_blocked) {
        P.rpm = 0.0f;
    }
    float airflow = P.rpm / RPM_MAX;
    float h = 0.4f * P.heat_pct;
    P.temp_c += dt * (h - (g0 + g1 * airflow) * (P.temp_c - P.ambient_c)) / tau;
    P.hum_pct = 55.0f - 0.8f * (P.temp_c - P.ambient_c);
    if (P.hum_pct < 10.0f) {
        P.hum_pct = 10.0f;
    }
}

/* ======================================================================
 *  W25Q64 on SPI2 (byte-level command decoder, 8 MB)
 * ====================================================================== */
#define W25_SIZE (8UL * 1024UL * 1024UL)
static struct {
    uint8_t  mem[W25_SIZE];
    bool     cs, wel;
    uint8_t  cmd;
    uint32_t phase, addr, busy;
} W;

static void w25_select(void *ctx, bool on)
{
    (void)ctx;
    if (on) {
        W.phase = 0;
    } else if (W.cmd == 0x02 || W.cmd == 0x20 || W.cmd == 0xC7) {
        W.wel = false;
        W.busy = 2;
    }
    W.cs = on;
}

static uint8_t w25_byte(uint8_t in)
{
    uint8_t out = 0xFF;
    if (W.phase == 0) {
        W.cmd = in;
        W.addr = 0;
        if (in == 0x06) { W.wel = true; }
        if (in == 0x04) { W.wel = false; }
        if (in == 0xC7 && W.wel) { memset(W.mem, 0xFF, W25_SIZE); }
    } else if ((W.cmd == 0x03 || W.cmd == 0x02 || W.cmd == 0x20) && W.phase <= 3) {
        W.addr = (W.addr << 8) | in;
        if (W.phase == 3 && W.cmd == 0x20 && W.wel) {
            memset(&W.mem[(W.addr % W25_SIZE) & ~0xFFFUL], 0xFF, 4096);
        }
    } else {
        switch (W.cmd) {
        case 0x9F: { static const uint8_t id[3] = { 0xEF, 0x40, 0x17 }; out = id[(W.phase - 1) % 3]; break; }
        case 0x05: out = (uint8_t)((W.busy ? 1 : 0) | (W.wel ? 2 : 0)); if (W.busy) { W.busy--; } break;
        case 0x03: out = W.mem[W.addr % W25_SIZE]; W.addr++; break;
        case 0x02:
            if (W.wel) {   /* NOR: program can only clear bits, wraps inside the page */
                uint32_t page = W.addr & ~0xFFUL;
                W.mem[(page | (W.addr & 0xFF)) % W25_SIZE] &= in;
                W.addr = page | ((W.addr + 1) & 0xFF);
            }
            break;
        default: break;
        }
    }
    W.phase++;
    return out;
}

static int w25_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx;
    if (P.flash_missing || !W.cs) {
        if (rx) { memset(rx, 0xFF, n); }   /* nobody drives MISO */
        return ERR_OK;
    }
    for (size_t i = 0; i < n; i++) {
        uint8_t o = w25_byte(tx ? tx[i] : 0xFF);
        if (rx) { rx[i] = o; }
    }
    return ERR_OK;
}

/* ======================================================================
 *  I2C1: SHT31 at 0x44, SSD1306 at 0x3C
 * ====================================================================== */
static uint8_t  s_sht_reply[6];
static uint8_t  s_oled_ram[8][128];       /* GDDRAM as the panel sees it */
static uint8_t  s_oled_col0, s_oled_col1, s_oled_page0, s_oled_page1, s_oled_col, s_oled_page;
static bool     s_oled_on;
static int      s_oled_expect;            /* bytes of a multi-byte command still expected */
static uint8_t  s_oled_cmd;

static void sht_prepare(void)
{
    float t = P.temp_c + noise(0.05f), h = P.hum_pct + noise(0.3f);
    uint16_t rt = (uint16_t)((t + 45.0f) * 65535.0f / 175.0f);
    uint16_t rh = (uint16_t)(h * 65535.0f / 100.0f);
    s_sht_reply[0] = (uint8_t)(rt >> 8); s_sht_reply[1] = (uint8_t)rt;
    s_sht_reply[2] = crc8_sensirion(&s_sht_reply[0], 2);
    s_sht_reply[3] = (uint8_t)(rh >> 8); s_sht_reply[4] = (uint8_t)rh;
    s_sht_reply[5] = crc8_sensirion(&s_sht_reply[3], 2);
}

static void oled_command(uint8_t b)
{
    if (s_oled_expect > 0) {
        if (s_oled_cmd == 0x21) {
            if (s_oled_expect == 2) { s_oled_col0 = b & 0x7F; } else { s_oled_col1 = b & 0x7F; s_oled_col = s_oled_col0; }
        } else if (s_oled_cmd == 0x22) {
            if (s_oled_expect == 2) { s_oled_page0 = b & 7; } else { s_oled_page1 = b & 7; s_oled_page = s_oled_page0; }
        }
        s_oled_expect--;
        return;
    }
    s_oled_cmd = b;
    switch (b) {
    case 0x21: case 0x22: s_oled_expect = 2; break;
    case 0xAF: s_oled_on = true; break;
    case 0xAE: s_oled_on = false; break;
    /* one-parameter commands used by the init sequence */
    case 0x20: case 0x81: case 0x8D: case 0xA8: case 0xD3: case 0xD5: case 0xD9: case 0xDA: case 0xDB:
        s_oled_expect = 1; s_oled_cmd = 0; break;
    default: break;
    }
}

static void oled_data(uint8_t b)
{
    s_oled_ram[s_oled_page][s_oled_col] = b;
    if (s_oled_col >= s_oled_col1) {        /* horizontal addressing mode */
        s_oled_col = s_oled_col0;
        s_oled_page = (s_oled_page >= s_oled_page1) ? s_oled_page0 : (uint8_t)(s_oled_page + 1);
    } else {
        s_oled_col++;
    }
}

static uint32_t s_i2c_tx, s_i2c_nack;

static int i2c_write(void *ctx, uint8_t addr, const uint8_t *d, size_t n)
{
    (void)ctx;
    if (addr == SHT3X_ADDR_DEFAULT && !P.sht_unplugged) {
        s_i2c_tx++;
        if (n == 2 && d[0] == 0x24) { sht_prepare(); }
        return ERR_OK;
    }
    if (addr == SSD1306_ADDR && !P.oled_unplugged) {
        s_i2c_tx++;
        if (n == 0) { return ERR_OK; }
        bool data = (d[0] & 0x40) != 0;
        for (size_t i = 1; i < n; i++) {
            if (data) { oled_data(d[i]); } else { oled_command(d[i]); }
        }
        return ERR_OK;
    }
    s_i2c_nack++;
    return ERR_IO;                          /* address NACK */
}

static int i2c_read(void *ctx, uint8_t addr, uint8_t *d, size_t n)
{
    (void)ctx;
    if (addr == SHT3X_ADDR_DEFAULT && !P.sht_unplugged) {
        s_i2c_tx++;
        memcpy(d, s_sht_reply, n < 6 ? n : 6);
        return ERR_OK;
    }
    s_i2c_nack++;
    return ERR_IO;
}

/* ======================================================================
 *  "Firmware": same data and steps as the RTOS tasks on the target
 * ====================================================================== */
static const i2c_bus_t s_i2c = { i2c_write, i2c_read, NULL };
static const spi_bus_t s_spi = { w25_xfer, w25_select, NULL };

static struct {
    fan_config_t cfg;
    /* measurements (sys_data_t.m on target) */
    float    temp_c, hum_pct, pot_percent, duty;
    bool     temp_valid, flash_ok, oled_ok;
    uint16_t rpm;
    uint32_t sensor_errors;
} D;

static app_fsm_t       s_fsm;
static fan_logic_t     s_logic;
static fault_monitor_t s_faults;
static w25qxx_t        s_w25;
static flash_dev_t     s_flash;
static datalog_t       s_log;
static sht3x_t         s_sht;
static ssd1306_t       s_oled;
static mb_slave_t      s_mb;
static cli_t           s_cli;
static bool            s_sht_present;
static uint32_t        s_sht_errors;
static uint32_t        s_link_sent, s_link_acked, s_link_cmds;
static bool            s_dirty;          /* settings changed but not saved */

static const char *fmt1(char *b, float v) { snprintf(b, 16, "%.1f", (double)v); return b; }

/* ======================================================================
 *  Two-tier watchdog: per-task deadlines (services/supervisor.c) + IWDG
 * ====================================================================== */
#define IWDG_TIMEOUT_MS 2000U
enum { SV_CONTROL, SV_SENSORS, SV_STORAGE, SV_DISPLAY, SV_LINK, SV_CONSOLE, SV_COUNT };
static const struct { const char *name; uint32_t timeout_ms; } SV[SV_COUNT] = {
    { "control", 500U }, { "sensors", 1000U }, { "storage", 70000U },
    { "display", 3000U }, { "link", 5000U }, { "console", 3000U },
};
static uint32_t    s_sv_kick[SV_COUNT];
static bool        s_sv_hung[SV_COUNT];   /* injected: task stuck in an endless loop */
static int         s_sv_late;             /* first task that missed its deadline, -1 = none */
static uint32_t    s_iwdg_fed;            /* last IWDG refresh */
static uint32_t    s_wdt_resets;          /* RTC backup register DR1: kept over a reset */
static const char *s_reset_cause = "POWER";
static bool        s_wdt_fired;

static void sv_kick(int id) { s_sv_kick[id] = s_ms; }

/* supervisor task (250 ms): feed the IWDG only while every task is on time */
static void supervisor_step(void)
{
    if (s_ms % 250U != 0U) {
        return;
    }
    for (int i = 0; s_sv_late < 0 && i < SV_COUNT; i++) {
        if (s_ms - s_sv_kick[i] > SV[i].timeout_ms) {
            s_sv_late = i;
            log_printf("WDG", "task '%s' missed its %lu ms deadline -> reset", SV[i].name,
                       (unsigned long)SV[i].timeout_ms);
        }
    }
    if (s_sv_late < 0) {
        s_iwdg_fed = s_ms;
    } else if (s_ms - s_iwdg_fed >= IWDG_TIMEOUT_MS) {
        s_wdt_fired = true;                 /* IWDG counter reached 0 */
    }
}

/* ---------------- control task body (100 ms) ---------------- */
static void control_step(uint32_t tick)
{
    const fan_config_t *c = &D.cfg;
    bool stall = c->stall_rpm > 0U && D.duty >= (c->min_duty_c10 / 10.0f) && D.duty > 5.0f && D.rpm < c->stall_rpm;
    float limit = c->over_temp_c100 / 100.0f;
    if (fault_active(&s_faults, FAULT_OVER_TEMP)) { limit -= 2.0f; }
    bool hot = D.temp_valid && D.temp_c > limit;
    bool sensor = !D.temp_valid && (s_fsm.state == APP_STATE_AUTO ||
                                    (s_fsm.state == APP_STATE_FAULT && s_fsm.resume == APP_STATE_AUTO));
    uint32_t before = fault_bits(&s_faults);
    fault_update(&s_faults, FAULT_FAN_STALL, stall,  TICK_MS);
    fault_update(&s_faults, FAULT_SENSOR,    sensor, TICK_MS);
    fault_update(&s_faults, FAULT_OVER_TEMP, hot,    TICK_MS);
    fault_update(&s_faults, FAULT_STORAGE,   !D.flash_ok, TICK_MS);
    uint32_t after = fault_bits(&s_faults);
    for (int i = 0; i < FAULT_COUNT; i++) {
        uint32_t bit = 1UL << i;
        if ((before ^ after) & bit) {
            log_printf("FAULT", "%s %s", fault_name((fault_id_t)i), (after & bit) ? "RAISED" : "cleared");
        }
    }
    uint32_t crit = after & CRITICAL_FAULTS;
    if (crit != 0U && s_fsm.state != APP_STATE_FAULT) {
        (void)app_fsm_handle(&s_fsm, APP_EV_FAULT_RAISED);
        log_printf("CTRL", "-> FAULT (mask 0x%02lX), output %u.%u %%", (unsigned long)crit,
                   c->fault_duty_c10 / 10U, c->fault_duty_c10 % 10U);
    } else if (crit == 0U && s_fsm.state == APP_STATE_FAULT) {
        (void)app_fsm_handle(&s_fsm, APP_EV_FAULT_CLEARED);
        log_printf("CTRL", "fault gone -> %s", app_state_name(s_fsm.state));
    }
    fan_inputs_t in = { D.temp_c, D.temp_valid, D.rpm, D.pot_percent };
    D.duty = fan_logic_step(&s_logic, c, s_fsm.state, &in, TICK_MS / 1000.0f);
    P.duty_out = D.duty;
    (void)tick;
}

static void post_event(app_event_t ev)
{
    if (app_fsm_handle(&s_fsm, ev)) {
        log_printf("CTRL", "-> %s", app_state_name(s_fsm.state));
    }
}

/* ---------------- sensors task body ---------------- */
static void sensors_step(void)
{
    /* knob + tach every 100 ms; tach as the 1 MHz input capture sees it */
    D.pot_percent += 0.3f * (P.pot_pct - D.pot_percent);
    if (P.rpm < 60.0f) {
        D.rpm = 0;
    } else {
        uint32_t period_us = (uint32_t)(60e6f / (P.rpm * 2.0f));   /* 2 pulses / rev */
        D.rpm = (uint16_t)(60000000UL / (period_us * 2UL));
    }
    if (s_ms % 1000U != 0U) {
        return;
    }
    if (!s_sht_present && s_ms % 5000U == 0U) {
        s_sht_present = (sht3x_soft_reset(&s_sht) == ERR_OK);
        if (s_sht_present) { log_printf("SENS", "SHT31 found"); }
    }
    sht3x_sample_t s;
    int rc = s_sht_present ? sht3x_measure(&s_sht, &s) : ERR_NOT_FOUND;
    if (rc == ERR_OK) {
        s_sht_errors = 0;
        D.temp_c = s.temp_c100 / 100.0f;
        D.hum_pct = s.hum_c100 / 100.0f;
        D.temp_valid = true;
    } else {
        D.sensor_errors++;
        if (s_sht_errors < 3) { s_sht_errors++; }
        if (s_sht_errors >= 3 && D.temp_valid) {
            D.temp_valid = false;
            s_sht_present = false;
            log_printf("SENS", "SHT31 lost");
        }
    }
}

/* ---------------- storage task body (1 s) ---------------- */
static uint32_t s_log_elapsed;
static void storage_step(void)
{
    if (s_ms % 1000U != 0U || !D.flash_ok) {
        return;
    }
    if (++s_log_elapsed < D.cfg.log_period_s) {
        return;
    }
    s_log_elapsed = 0;
    datalog_sample_t s = {
        .timestamp = s_unix0 + s_ms / 1000U,
        .temp_c100 = D.temp_valid ? (int16_t)(D.temp_c * 100.0f) : INT16_MIN,
        .hum_c100  = D.temp_valid ? (uint16_t)(D.hum_pct * 100.0f) : 0U,
        .rpm       = D.rpm,
        .duty_c10  = (uint16_t)(D.duty * 10.0f + 0.5f),
        .state     = (uint8_t)s_fsm.state,
        .faults    = (uint8_t)fault_bits(&s_faults),
    };
    int rc = datalog_append(&s_log, &s);
    if (rc != ERR_OK) { log_printf("FLASH", "log append failed rc=%d", rc); }
}

/* ---------------- display task body (500 ms) ---------------- */
static void display_step(void)
{
    if (s_ms % 500U != 0U || !D.oled_ok) {
        return;
    }
    char line[24], a[16], b[16];
    ssd1306_clear(&s_oled);
    snprintf(line, sizeof line, "FAN CTRL    %-6s", app_state_name(s_fsm.state));
    ssd1306_text(&s_oled, 0, 0, line);
    if (D.temp_valid) {
        snprintf(line, sizeof line, "T %sC H %s%%", fmt1(a, D.temp_c), fmt1(b, D.hum_pct));
    } else {
        snprintf(line, sizeof line, "T --.-C  (sensor?)");
    }
    ssd1306_text(&s_oled, 0, 2, line);
    snprintf(line, sizeof line, "SP %sC  %u rpm", fmt1(a, D.cfg.setpoint_c100 / 100.0f), D.rpm);
    ssd1306_text(&s_oled, 0, 3, line);
    snprintf(line, sizeof line, "Out %s%%", fmt1(a, D.duty));
    ssd1306_text(&s_oled, 0, 4, line);
    ssd1306_bar(&s_oled, 0, 42, 128, 8, (int)(D.duty + 0.5f));
    uint32_t f = fault_bits(&s_faults);
    if (f != 0U) {
        for (int i = 0; i < FAULT_COUNT; i++) {
            if (f & (1UL << i)) { snprintf(line, sizeof line, "! %s", fault_name((fault_id_t)i)); break; }
        }
    } else {
        uint32_t up = s_ms / 1000U;
        snprintf(line, sizeof line, "up %luh%02lum", (unsigned long)(up / 3600U), (unsigned long)((up / 60U) % 60U));
    }
    ssd1306_text(&s_oled, 0, 7, line);
    if (ssd1306_flush(&s_oled) != ERR_OK) {
        D.oled_ok = false;
        log_printf("OLED", "I2C error, display off");
    }
}

/* ---------------- radio link task body (100 ms) ---------------- */
static struct {
    bool     waiting;
    uint32_t wait_seq, wait_slot, sent_at, next_at;
    char     rx[TLM_LINE_MAX];
    size_t   rx_len;
    char     gw_pending[TLM_LINE_MAX];     /* gateway ACK in flight */
    uint32_t gw_at;
} L;

static void link_apply_cmd(const tlm_msg_t *m)
{
    s_link_cmds++;
    if (strcmp(m->name, "mode") == 0 && m->value >= 0 && m->value <= 2) {
        static const app_event_t ev[3] = { APP_EV_MODE_OFF, APP_EV_MODE_MANUAL, APP_EV_MODE_AUTO };
        post_event(ev[m->value]);
    } else if (strcmp(m->name, "duty") == 0 && m->value >= 0 && m->value <= 1000) {
        D.cfg.manual_duty_c10 = (uint16_t)m->value; D.cfg.use_pot = 0U;
    } else if (strcmp(m->name, "setpoint") == 0 && m->value >= 0 && m->value <= 8000) {
        D.cfg.setpoint_c100 = (int16_t)m->value;
    } else {
        log_printf("LINK", "unknown cmd %s=%ld", m->name, (long)m->value);
        return;
    }
    log_printf("LINK", "cmd %s=%ld", m->name, (long)m->value);
}

static void link_rx_line(const char *line)
{
    tlm_msg_t m;
    if (tlm_parse(line, &m) != ERR_OK || m.node != D.cfg.node_id) {
        return;
    }
    if (m.kind == TLM_ACK && L.waiting && m.seq == L.wait_seq) {
        (void)datalog_mark_sent(&s_log, L.wait_slot);
        s_link_acked++;
        L.waiting = false;
        L.next_at = s_ms + 500U;
    } else if (m.kind == TLM_CMD) {
        link_apply_cmd(&m);
    }
}

static void link_step(void)
{
    /* gateway side: deliver a pending $ACK after the air time */
    if (L.gw_pending[0] != '\0' && s_ms >= L.gw_at) {
        out_put(&s_air, "<- ");
        out_put(&s_air, L.gw_pending);
        link_rx_line(L.gw_pending);
        L.gw_pending[0] = '\0';
    }
    if (L.waiting && s_ms - L.sent_at > 2000U) {
        L.waiting = false;
        L.next_at = s_ms + D.cfg.tlm_period_s * 1000U;
    }
    if (L.waiting || (int32_t)(s_ms - L.next_at) < 0) {
        return;
    }
    datalog_sample_t s;
    char line[TLM_LINE_MAX];
    if (D.flash_ok) {
        if (datalog_next_unsent(&s_log, &L.wait_slot, &L.wait_seq, &s) != ERR_OK) {
            L.next_at = s_ms + 1000U;
            return;
        }
        L.waiting = true;
    } else {
        s = (datalog_sample_t){ s_unix0 + s_ms / 1000U, D.temp_valid ? (int16_t)(D.temp_c * 100.0f) : INT16_MIN,
                                (uint16_t)(D.hum_pct * 100.0f), D.rpm, (uint16_t)(D.duty * 10.0f + 0.5f),
                                (uint8_t)s_fsm.state, (uint8_t)fault_bits(&s_faults) };
        L.wait_seq = 0;
        L.next_at = s_ms + D.cfg.tlm_period_s * 1000U;
    }
    if (tlm_encode_sample(line, sizeof line, D.cfg.node_id, L.wait_seq, &s) > 0U) {
        out_put(&s_air, "-> ");
        out_put(&s_air, line);
        s_link_sent++;
        if (P.gateway_online && L.waiting) {      /* gateway stores it and answers */
            char body[48];
            int n = snprintf(body, sizeof body, "ACK,%u,%lu", D.cfg.node_id, (unsigned long)L.wait_seq);
            snprintf(L.gw_pending, sizeof L.gw_pending, "$%s*%02X\r\n", body, tlm_checksum(body, (size_t)n));
            L.gw_at = s_ms + 300U;
        }
    }
    L.sent_at = s_ms;
}

/* ======================================================================
 *  Modbus register map (same as services/modbus_server.c)
 * ====================================================================== */
static mb_exception_t mb_read(void *ctx, mb_table_t table, uint16_t addr, uint16_t *v)
{
    (void)ctx;
    const fan_config_t *c = &D.cfg;
    uint32_t up = s_ms / 1000U, nlog = datalog_count(&s_log);
    if (table == MB_TABLE_INPUT) {
        switch (addr) {
        case 0:  *v = D.temp_valid ? (uint16_t)(int16_t)(D.temp_c * 100.0f) : 0x8000U; break;
        case 1:  *v = (uint16_t)(D.hum_pct * 100.0f); break;
        case 2:  *v = D.rpm; break;
        case 3:  *v = (uint16_t)(D.duty * 10.0f + 0.5f); break;
        case 4:  *v = (uint16_t)s_fsm.state; break;
        case 5:  *v = (uint16_t)fault_bits(&s_faults); break;
        case 6:  *v = (uint16_t)(D.pot_percent * 10.0f); break;
        case 7:  *v = (uint16_t)(up >> 16); break;
        case 8:  *v = (uint16_t)up; break;
        case 9:  *v = (uint16_t)(nlog >> 16); break;
        case 10: *v = (uint16_t)nlog; break;
        case 11: *v = FW_VERSION_REG; break;
        default: return MB_EX_ILLEGAL_ADDRESS;
        }
        return MB_EX_NONE;
    }
    switch (addr) {
    case 0:  *v = (uint16_t)s_fsm.state; break;
    case 1:  *v = c->manual_duty_c10; break;
    case 2:  *v = (uint16_t)c->setpoint_c100; break;
    case 3:  *v = (uint16_t)(c->kp * 100.0f + 0.5f); break;
    case 4:  *v = (uint16_t)(c->ki * 100.0f + 0.5f); break;
    case 5:  *v = (uint16_t)(c->kd * 100.0f + 0.5f); break;
    case 6:  *v = c->min_duty_c10; break;
    case 7:  *v = c->fault_duty_c10; break;
    case 8:  *v = (uint16_t)c->over_temp_c100; break;
    case 9:  *v = c->ramp_c10_per_s; break;
    case 10: *v = c->log_period_s; break;
    case 11: *v = c->tlm_period_s; break;
    case 12: *v = c->modbus_addr; break;
    case 13: *v = c->use_pot; break;
    case 14: *v = c->stall_rpm; break;
    case 20: *v = 0U; break;
    default: return MB_EX_ILLEGAL_ADDRESS;
    }
    return MB_EX_NONE;
}

static mb_exception_t mb_write(void *ctx, uint16_t addr, uint16_t v)
{
    (void)ctx;
    fan_config_t *c = &D.cfg;
    mb_exception_t ex = MB_EX_NONE;
    if (addr == 0) {
        static const app_event_t ev[3] = { APP_EV_MODE_OFF, APP_EV_MODE_MANUAL, APP_EV_MODE_AUTO };
        if (v > 2U) { return MB_EX_ILLEGAL_VALUE; }
        post_event(ev[v]);
        return MB_EX_NONE;
    }
    if (addr == 20) {
        switch (v) {
        case 1: s_dirty = false; log_printf("CFG", "saved (Modbus)"); return MB_EX_NONE;
        case 2: fault_clear_all(&s_faults); log_printf("FAULT", "faults cleared (Modbus)"); return MB_EX_NONE;
        case 3: log_printf("SYS", "reboot requested (ignored in simulator)"); return MB_EX_NONE;
        default: return MB_EX_ILLEGAL_VALUE;
        }
    }
    switch (addr) {
    case 1:  if (v <= 1000U) { c->manual_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 2:  if (v <= 8000U) { c->setpoint_c100 = (int16_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 3:  c->kp = v / 100.0f; break;
    case 4:  c->ki = v / 100.0f; break;
    case 5:  c->kd = v / 100.0f; break;
    case 6:  if (v <= 500U)  { c->min_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 7:  if (v <= 1000U) { c->fault_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 8:  if (v >= 2000U && v <= 12000U) { c->over_temp_c100 = (int16_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 9:  if (v >= 10U && v <= 1000U) { c->ramp_c10_per_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 10: if (v >= 1U && v <= 3600U) { c->log_period_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 11: if (v >= 5U && v <= 3600U) { c->tlm_period_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 12: if (v >= 1U && v <= 247U)  { c->modbus_addr = (uint8_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 13: c->use_pot = (v != 0U); break;
    case 14: c->stall_rpm = v; break;
    default: ex = MB_EX_ILLEGAL_ADDRESS; break;
    }
    if (ex == MB_EX_NONE) { s_dirty = true; }
    return ex;
}

static const mb_callbacks_t s_mb_cb = { mb_read, mb_write, NULL };

static uint8_t s_mb_req[MB_FRAME_MAX], s_mb_resp[MB_FRAME_MAX];
EXPORT(sim_mb_req_buf)  uint8_t *sim_mb_req_buf(void)  { return s_mb_req; }
EXPORT(sim_mb_resp_buf) uint8_t *sim_mb_resp_buf(void) { return s_mb_resp; }
/** Feed one RTU frame (already in sim_mb_req_buf). @return reply length, 0 = no reply. */
EXPORT(sim_modbus) int sim_modbus(int len)
{
    return (int)mb_slave_process(&s_mb, s_mb_req, (size_t)len, s_mb_resp);
}

/* ======================================================================
 *  Console (subset of services/console.c)
 * ====================================================================== */
static void cli_write(const char *s) { out_put(&s_console, s); }

static int cmd_status(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    char a[16], b[16];
    cli_printf(c, "state    %s\r\n", app_state_name(s_fsm.state));
    if (D.temp_valid) {
        cli_printf(c, "temp     %s C   hum %s %%\r\n", fmt1(a, D.temp_c), fmt1(b, D.hum_pct));
    } else {
        cli_printf(c, "temp     -- (no sensor)\r\n");
    }
    cli_printf(c, "setpoint %s C\r\n", fmt1(a, D.cfg.setpoint_c100 / 100.0f));
    cli_printf(c, "output   %s %%   fan %u rpm\r\n", fmt1(a, D.duty), D.rpm);
    cli_printf(c, "knob     %s %%\r\n", fmt1(a, D.pot_percent));
    cli_printf(c, "faults   0x%02lX\r\n", (unsigned long)fault_bits(&s_faults));
    cli_printf(c, "log      %lu records\r\n", (unsigned long)datalog_count(&s_log));
    cli_printf(c, "uptime   %lu s\r\n", (unsigned long)(s_ms / 1000U));
    return 0;
}

static int cmd_mode(cli_t *c, int argc, char *argv[])
{
    static const struct { const char *n; app_event_t ev; } map[] = {
        { "off", APP_EV_MODE_OFF }, { "manual", APP_EV_MODE_MANUAL }, { "auto", APP_EV_MODE_AUTO },
    };
    for (size_t i = 0; argc == 2 && i < 3; i++) {
        if (strcmp(argv[1], map[i].n) == 0) { post_event(map[i].ev); return 0; }
    }
    cli_printf(c, "usage: mode off|manual|auto\r\n");
    return -1;
}

static int cmd_duty(cli_t *c, int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "knob") == 0) {
        D.cfg.use_pot = 1U;
        cli_printf(c, "manual output follows the knob\r\n");
        return 0;
    }
    float v = argc == 2 ? strtof(argv[1], NULL) : -1.0f;
    if (v < 0.0f || v > 100.0f) { cli_printf(c, "usage: duty <0..100> | knob\r\n"); return -1; }
    D.cfg.manual_duty_c10 = (uint16_t)(v * 10.0f + 0.5f);
    D.cfg.use_pot = 0U;
    s_dirty = true;
    cli_printf(c, "manual duty %u.%u %%\r\n", D.cfg.manual_duty_c10 / 10U, D.cfg.manual_duty_c10 % 10U);
    return 0;
}

static int cmd_setpoint(cli_t *c, int argc, char *argv[])
{
    float v = argc == 2 ? strtof(argv[1], NULL) : -1.0f;
    if (v < 0.0f || v > 80.0f) { cli_printf(c, "usage: setpoint <0..80>\r\n"); return -1; }
    D.cfg.setpoint_c100 = (int16_t)(v * 100.0f);
    s_dirty = true;
    return 0;
}

static int cmd_pid(cli_t *c, int argc, char *argv[])
{
    char a[16], b[16], d[16];
    if (argc == 4) {
        D.cfg.kp = strtof(argv[1], NULL);
        D.cfg.ki = strtof(argv[2], NULL);
        D.cfg.kd = strtof(argv[3], NULL);
        (void)fan_config_sanitize(&D.cfg);
        s_dirty = true;
    } else if (argc != 1) {
        cli_printf(c, "usage: pid [kp ki kd]\r\n");
        return -1;
    }
    snprintf(a, sizeof a, "%.2f", (double)D.cfg.kp);
    snprintf(b, sizeof b, "%.2f", (double)D.cfg.ki);
    snprintf(d, sizeof d, "%.2f", (double)D.cfg.kd);
    cli_printf(c, "kp %s  ki %s  kd %s\r\n", a, b, d);
    return 0;
}

static int cmd_log(cli_t *c, int argc, char *argv[])
{
    if (argc >= 2 && strcmp(argv[1], "dump") == 0) {
        uint32_t n = datalog_count(&s_log), want = argc == 3 ? (uint32_t)strtoul(argv[2], NULL, 10) : 10U;
        uint32_t first = n > want ? n - want : 0U;
        cli_printf(c, "seq,time,temp_c,hum,rpm,duty,state,faults\r\n");
        for (uint32_t i = first; i < n; i++) {
            datalog_sample_t s;
            uint32_t seq;
            if (datalog_read(&s_log, i, &s, &seq) == ERR_OK) {
                cli_printf(c, "%lu,%lu,%d.%02d,%u,%u,%u,%u,%u\r\n", (unsigned long)seq, (unsigned long)s.timestamp,
                           s.temp_c100 / 100, abs(s.temp_c100 % 100), s.hum_c100 / 100, s.rpm, s.duty_c10 / 10,
                           s.state, s.faults);
            }
        }
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        int rc = datalog_format(&s_log);
        cli_printf(c, "log cleared rc=%d\r\n", rc);
        return 0;
    }
    cli_printf(c, "%lu records\r\n", (unsigned long)datalog_count(&s_log));
    return 0;
}

static int cmd_flash(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    uint32_t id = 0;
    int rc = w25qxx_read_jedec(&s_w25, &id);
    cli_printf(c, "JEDEC 0x%06lX rc=%d  size %lu KB  %s\r\n", (unsigned long)id, rc,
               (unsigned long)(s_w25.size / 1024U), D.flash_ok ? "OK" : "NOT USED");
    return 0;
}

static int cmd_i2c(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    cli_printf(c, "I2C1 scan:");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_write(NULL, a, NULL, 0) == ERR_OK) { cli_printf(c, " 0x%02X", a); }
    }
    cli_printf(c, "\r\n");
    return 0;
}

static int cmd_fault(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    fault_clear_all(&s_faults);
    cli_printf(c, "faults cleared\r\n");
    return 0;
}

static int cmd_wdt(cli_t *c, int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "test") == 0) {
        cli_printf(c, "console task hangs now: supervisor reports it after %lu ms, IWDG resets %lu ms later\r\n",
                   (unsigned long)SV[SV_CONSOLE].timeout_ms, (unsigned long)IWDG_TIMEOUT_MS);
        s_sv_hung[SV_CONSOLE] = true;
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "clear") == 0) {
        s_wdt_resets = 0;
    } else if (argc != 1) {
        cli_printf(c, "usage: wdt | wdt test | wdt clear\r\n");
        return 1;
    }
    cli_printf(c, "IWDG %lu ms   watchdog resets %lu   last reset %s\r\n", (unsigned long)IWDG_TIMEOUT_MS,
               (unsigned long)s_wdt_resets, s_reset_cause);
    cli_printf(c, "task       deadline   last kick  state\r\n");
    for (int i = 0; i < SV_COUNT; i++) {
        uint32_t age = s_ms - s_sv_kick[i];
        cli_printf(c, "%-10s %6lu ms %8lu ms  %s\r\n", SV[i].name, (unsigned long)SV[i].timeout_ms,
                   (unsigned long)age, age > SV[i].timeout_ms ? "LATE" : "ok");
    }
    return 0;
}

static int cmd_stats(cli_t *c, int argc, char *argv[])
{
    (void)argc; (void)argv;
    cli_printf(c, "modbus  frames %lu  crc errors %lu  exceptions %lu\r\n",
               (unsigned long)s_mb.rx_frames, (unsigned long)s_mb.crc_errors, (unsigned long)s_mb.exceptions);
    cli_printf(c, "lora    sent %lu  acked %lu  commands %lu\r\n",
               (unsigned long)s_link_sent, (unsigned long)s_link_acked, (unsigned long)s_link_cmds);
    cli_printf(c, "i2c     transfers %lu  nack %lu   sensor errors %lu\r\n",
               (unsigned long)s_i2c_tx, (unsigned long)s_i2c_nack, (unsigned long)D.sensor_errors);
    return 0;
}

static int cmd_cfg(cli_t *c, int argc, char *argv[])
{
    if (argc == 2 && strcmp(argv[1], "save") == 0) { s_dirty = false; cli_printf(c, "saved\r\n"); return 0; }
    if (argc == 2 && strcmp(argv[1], "default") == 0) {
        fan_config_defaults(&D.cfg); s_dirty = true; cli_printf(c, "defaults loaded\r\n"); return 0;
    }
    const fan_config_t *k = &D.cfg;
    cli_printf(c, "modbus_addr %u  node %u  start_mode %u\r\n", k->modbus_addr, k->node_id, k->start_mode);
    cli_printf(c, "min_duty %u  fault_duty %u  ramp %u (x0.1 %%/s)\r\n", k->min_duty_c10, k->fault_duty_c10, k->ramp_c10_per_s);
    cli_printf(c, "over_temp %d (x0.01 C)  stall_rpm %u\r\n", k->over_temp_c100, k->stall_rpm);
    cli_printf(c, "log %u s  telemetry %u s  use_knob %u  %s\r\n", k->log_period_s, k->tlm_period_s, k->use_pot,
               s_dirty ? "(not saved)" : "");
    return 0;
}

static const cli_cmd_t s_cmds[] = {
    { "status",   "show measurements and state",       cmd_status },
    { "mode",     "off | manual | auto",               cmd_mode },
    { "duty",     "<0..100> manual output, or 'knob'", cmd_duty },
    { "setpoint", "<C> auto-mode temperature",         cmd_setpoint },
    { "pid",      "[kp ki kd] show / set PID gains",   cmd_pid },
    { "cfg",      "show | save | default",             cmd_cfg },
    { "log",      "count | dump [n] (CSV) | clear",    cmd_log },
    { "flash",    "SPI flash info",                    cmd_flash },
    { "i2c",      "scan the I2C bus",                  cmd_i2c },
    { "fault",    "clear latched faults",              cmd_fault },
    { "stats",    "Modbus / LoRa / I2C counters",      cmd_stats },
    { "wdt",      "watchdog status | test | clear",    cmd_wdt },
};

static char s_line_in[CLI_LINE_MAX];
EXPORT(sim_cli_buf) char *sim_cli_buf(void) { return s_line_in; }
/** Run the command line already written to sim_cli_buf. */
EXPORT(sim_cli) void sim_cli(void)
{
    char line[CLI_LINE_MAX];
    strncpy(line, s_line_in, sizeof line - 1);
    line[sizeof line - 1] = '\0';
    out_put(&s_console, "> ");
    out_put(&s_console, line);
    out_put(&s_console, "\r\n");
    if (!s_sv_hung[SV_CONSOLE]) {           /* a hung console task reads nothing */
        (void)cli_execute(&s_cli, line);
    }
}

/** A line typed on the gateway side (e.g. $CMD,...) sent over the air. */
EXPORT(sim_lora_send) void sim_lora_send(void)
{
    char line[CLI_LINE_MAX + 4];
    snprintf(line, sizeof line, "%s", s_line_in);
    out_put(&s_air, "<- ");
    out_put(&s_air, line);
    out_put(&s_air, "\r\n");
    link_rx_line(line);
}

/* ======================================================================
 *  Life cycle and UI accessors
 * ====================================================================== */
static void firmware_boot(void)
{
    memset(&D, 0, sizeof D);
    memset(&L, 0, sizeof L);
    s_ms = 0;
    s_log_elapsed = 0;
    s_sht_errors = 0;
    s_link_sent = s_link_acked = s_link_cmds = 0;
    s_i2c_tx = s_i2c_nack = 0;
    memset(s_oled_ram, 0, sizeof s_oled_ram);
    s_oled_on = false;

    memset(s_sv_kick, 0, sizeof s_sv_kick);
    memset(s_sv_hung, 0, sizeof s_sv_hung);
    s_sv_late = -1;
    s_iwdg_fed = 0;
    s_wdt_fired = false;

    log_printf("SYS", "fan controller simulator, firmware 1.0 (STM32G0B1RE model), reset: %s", s_reset_cause);
    log_printf("WDG", "IWDG %lu ms, %lu watchdog reset(s) since power-up", (unsigned long)IWDG_TIMEOUT_MS,
               (unsigned long)s_wdt_resets);
    fan_config_defaults(&D.cfg);
    D.cfg.stall_rpm = 300;          /* stall detection on, so it can be demonstrated */

    D.flash_ok = (w25qxx_init(&s_w25, &s_spi, no_delay) == ERR_OK);
    if (D.flash_ok) {
        w25qxx_as_flash(&s_w25, &s_flash);
        uint32_t nsec = (s_flash.size - CFG_LOG_BASE) / s_flash.sector_size;
        if (nsec > 2032U) { nsec = 2032U; }
        int rc = datalog_init(&s_log, &s_flash, CFG_LOG_BASE, nsec);
        log_printf("FLASH", "W25Q64 JEDEC 0x%06lX, log %lu records (rc=%d)", (unsigned long)s_w25.jedec_id,
                   (unsigned long)datalog_count(&s_log), rc);
    } else {
        memset(&s_log, 0, sizeof s_log);
        log_printf("FLASH", "SPI flash NOT found - running without log");
    }
    s_sht_present = (sht3x_init(&s_sht, &s_i2c, SHT3X_ADDR_DEFAULT, no_delay) == ERR_OK);
    log_printf("SENS", "SHT31 %s", s_sht_present ? "found" : "NOT found");
    D.oled_ok = (ssd1306_init(&s_oled, &s_i2c, SSD1306_ADDR) == ERR_OK);
    log_printf("OLED", "SSD1306 %s", D.oled_ok ? "found" : "NOT found");

    mb_slave_init(&s_mb, D.cfg.modbus_addr, &s_mb_cb);
    cli_init(&s_cli, s_cmds, sizeof s_cmds / sizeof s_cmds[0], cli_write, false);
    fault_init(&s_faults);
    fault_config(&s_faults, FAULT_FAN_STALL, 5000U,    0U, true);
    fault_config(&s_faults, FAULT_SENSOR,    3000U, 2000U, false);
    fault_config(&s_faults, FAULT_OVER_TEMP, 2000U, 5000U, false);
    fault_config(&s_faults, FAULT_STORAGE,      0U, 1000U, false);
    app_fsm_init(&s_fsm, (app_state_t)D.cfg.start_mode);
    fan_logic_init(&s_logic, &D.cfg);
    L.next_at = 3000U;
    log_printf("CTRL", "start in %s", app_state_name(s_fsm.state));
}

/** Power-on: plant at room temperature, empty flash. */
EXPORT(sim_init) void sim_init(uint32_t unix_now)
{
    memset(&P, 0, sizeof P);
    P.ambient_c = 28.0f;
    P.heat_pct = 30.0f;
    P.temp_c = P.ambient_c;
    P.hum_pct = 55.0f;
    P.gateway_online = true;
    P.pot_pct = 50.0f;
    memset(W.mem, 0xFF, W25_SIZE);
    s_unix0 = unix_now;
    s_wdt_resets = 0;                       /* backup domain lost with power */
    s_reset_cause = "POWER";
    sim_outputs_clear();
    firmware_boot();
}

/** Reset button: the flash keeps its content (the log survives). */
EXPORT(sim_reset) void sim_reset(void)
{
    s_unix0 += s_ms / 1000U;
    s_reset_cause = "PIN";
    out_put(&s_console, "\r\n--- RESET ---\r\n");
    firmware_boot();
}

/** Advance simulated time by n ticks of 100 ms. A hung task skips its step. */
EXPORT(sim_run) void sim_run(int ticks)
{
    for (int i = 0; i < ticks; i++) {
        plant_step(TICK_MS / 1000.0f);
        s_ms += TICK_MS;
        if (!s_sv_hung[SV_SENSORS]) { sensors_step(); sv_kick(SV_SENSORS); }
        if (!s_sv_hung[SV_CONTROL]) { control_step(s_ms / TICK_MS); sv_kick(SV_CONTROL); }
        if (!s_sv_hung[SV_STORAGE]) { storage_step(); sv_kick(SV_STORAGE); }
        if (!s_sv_hung[SV_DISPLAY]) { display_step(); sv_kick(SV_DISPLAY); }
        if (!s_sv_hung[SV_LINK])    { link_step(); sv_kick(SV_LINK); }
        if (!s_sv_hung[SV_CONSOLE]) { sv_kick(SV_CONSOLE); }
        supervisor_step();
        if (s_wdt_fired) {
            s_wdt_resets++;
            s_reset_cause = "IWDG";
            s_unix0 += s_ms / 1000U;
            out_put(&s_console, "\r\n--- IWDG RESET ---\r\n");
            firmware_boot();
        }
    }
}

EXPORT(sim_button) void sim_button(void) { post_event(APP_EV_BUTTON); }

/* plant / fault injection knobs; id selects the input */
EXPORT(sim_set) void sim_set(int id, float v)
{
    switch (id) {
    case 0: P.ambient_c = v; break;
    case 1: P.heat_pct = v; break;
    case 2: P.pot_pct = v; break;
    case 3: P.fan_blocked = v != 0.0f; break;
    case 4: P.sht_unplugged = v != 0.0f; break;
    case 5: P.oled_unplugged = v != 0.0f; break;
    case 6: P.flash_missing = v != 0.0f; break;
    case 7: P.gateway_online = v != 0.0f; break;
    case 8: if (v >= 0.0f && v < SV_COUNT) { s_sv_hung[(int)v] = true; } break;   /* hang task #v */
    default: break;
    }
}

/* everything the UI plots, by id */
EXPORT(sim_get) float sim_get(int id)
{
    switch (id) {
    case 0:  return P.temp_c;                       /* true temperature        */
    case 1:  return D.temp_valid ? D.temp_c : -999; /* what the MCU measured   */
    case 2:  return D.cfg.setpoint_c100 / 100.0f;
    case 3:  return D.duty;
    case 4:  return (float)D.rpm;
    case 5:  return (float)s_fsm.state;
    case 6:  return (float)fault_bits(&s_faults);
    case 7:  return D.hum_pct;
    case 8:  return (float)datalog_count(&s_log);
    case 9:  return (float)(s_ms / 1000U);
    case 10: return D.pot_percent;
    case 11: return (fault_bits(&s_faults) & CRITICAL_FAULTS) ? 1.0f : 0.0f;   /* relay */
    case 12: {                                         /* LED, as control.c blinks it */
        uint32_t t = s_ms / TICK_MS;
        switch (s_fsm.state) {
        case APP_STATE_AUTO:   return 1.0f;
        case APP_STATE_MANUAL: return (t / 5U) % 2U == 0U;
        case APP_STATE_FAULT:  return t % 2U == 0U;
        default:               return t % 20U == 0U;
        }
    }
    case 13: return D.duty / 10.0f;                 /* 0-10 V output           */
    case 14: return (float)s_link_sent;
    case 15: return (float)s_link_acked;
    case 16: return (float)(s_log.count - (s_log.unsent >= s_log.tail ? s_log.unsent - s_log.tail
                                           : s_log.total_slots - s_log.tail + s_log.unsent));
    case 17: return s_oled_on && D.oled_ok ? 1.0f : 0.0f;
    case 18: return D.cfg.modbus_addr;
    case 19: return (float)s_wdt_resets;
    case 20: return s_sv_late >= 0 ? 1.0f : 0.0f;   /* supervisor stopped feeding IWDG */
    default: return 0.0f;
    }
}

EXPORT(sim_oled) const uint8_t *sim_oled(void) { return &s_oled_ram[0][0]; }
