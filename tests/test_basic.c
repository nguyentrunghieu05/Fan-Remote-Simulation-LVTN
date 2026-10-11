#include "test.h"
#include <string.h>
#include "ring_buffer.h"
#include "crc.h"
#include "pid.h"
#include "ramp.h"
#include "fault.h"
#include "timeconv.h"
#include "cli.h"

static void rb_basic(void)
{
    uint8_t mem[8]; ring_buffer_t rb; uint8_t b;
    CHECK(!rb_init(&rb, mem, 6));                       /* not power of two */
    CHECK(rb_init(&rb, mem, 8));
    CHECK_EQ(rb_count(&rb), 0);
    CHECK_EQ(rb_write(&rb, (const uint8_t *)"abcdefghij", 10), 7);   /* 1 slot kept free */
    CHECK(!rb_put(&rb, 'x'));
    CHECK(rb_get(&rb, &b)); CHECK_EQ(b, 'a');
    CHECK(rb_put(&rb, 'z'));                            /* wraps around */
    uint8_t out[8]; size_t n = rb_read(&rb, out, 8);
    CHECK_EQ(n, 7); CHECK(memcmp(out, "bcdefgz", 7) == 0);
    CHECK(!rb_get(&rb, &b));
}
void suite_ring_buffer(void) { RUN(rb_basic); }

static void crc_vectors(void)
{
    const uint8_t s[] = "123456789";
    CHECK_EQ(crc16_modbus(s, 9), 0x4B37);               /* catalogued check values */
    CHECK_EQ(crc32(s, 9), 0xCBF43926UL);
    const uint8_t sens[] = { 0xBE, 0xEF };
    CHECK_EQ(crc8_sensirion(sens, 2), 0x92);             /* example from the SHT3x datasheet */
    CHECK_EQ(crc32_update(crc32(s, 4), s + 4, 5), 0xCBF43926UL);  /* incremental */
}
void suite_crc(void) { RUN(crc_vectors); }

static void pid_cooling(void)
{
    pid_ctrl_t p;
    pid_init(&p, 10.0f, 1.0f, 0.0f, 0.0f, 100.0f, true);
    CHECK_NEAR(pid_update(&p, 30.0f, 30.0f, 0.1f), 0.0f, 1e-4);   /* at set point */
    pid_reset(&p);
    CHECK(pid_update(&p, 30.0f, 33.0f, 0.1f) > 29.0f);           /* hot -> fan up */
    pid_reset(&p);
    CHECK_NEAR(pid_update(&p, 30.0f, 25.0f, 0.1f), 0.0f, 1e-4);   /* cold -> 0, clamped */
}
static void pid_antiwindup(void)
{
    pid_ctrl_t p;
    pid_init(&p, 10.0f, 5.0f, 0.0f, 0.0f, 100.0f, true);
    for (int i = 0; i < 1000; i++) (void)pid_update(&p, 30.0f, 50.0f, 0.1f);  /* saturated */
    CHECK(p.integral <= 100.0f + 1e-3);
    /* once temperature drops below set point, output must leave 100 % quickly */
    float out = 100.0f;
    int steps = 0;
    while (out > 0.0f && steps < 200) { out = pid_update(&p, 30.0f, 28.0f, 0.1f); steps++; }
    CHECK(steps < 120);
}
static void ramp_limits(void)
{
    ramp_t r; ramp_init(&r, 0.0f, 20.0f);
    CHECK_NEAR(ramp_step(&r, 100.0f, 0.1f), 2.0f, 1e-4);
    for (int i = 0; i < 100; i++) ramp_step(&r, 100.0f, 0.1f);
    CHECK_NEAR(r.value, 100.0f, 1e-3);
    CHECK_NEAR(ramp_step(&r, 0.0f, 0.5f), 90.0f, 1e-3);
}
void suite_pid_ramp(void) { RUN(pid_cooling); RUN(pid_antiwindup); RUN(ramp_limits); }

static void fault_qualify(void)
{
    fault_monitor_t m; fault_init(&m);
    fault_config(&m, FAULT_SENSOR, 300, 1000, false);
    fault_config(&m, FAULT_FAN_STALL, 3000, 0, true);
    CHECK(!fault_update(&m, FAULT_SENSOR, true, 100));
    CHECK(!fault_update(&m, FAULT_SENSOR, false, 100));   /* glitch resets the timer */
    fault_update(&m, FAULT_SENSOR, true, 100);
    fault_update(&m, FAULT_SENSOR, true, 100);
    CHECK(fault_update(&m, FAULT_SENSOR, true, 100));     /* 300 ms continuous -> trip */
    CHECK(fault_active(&m, FAULT_SENSOR));
    for (int i = 0; i < 9; i++) fault_update(&m, FAULT_SENSOR, false, 100);
    CHECK(fault_active(&m, FAULT_SENSOR));
    CHECK(fault_update(&m, FAULT_SENSOR, false, 100));    /* 1 s clean -> release */
    CHECK_EQ(fault_bits(&m), 0);

    for (int i = 0; i < 30; i++) fault_update(&m, FAULT_FAN_STALL, true, 100);
    CHECK(fault_active(&m, FAULT_FAN_STALL));
    for (int i = 0; i < 100; i++) fault_update(&m, FAULT_FAN_STALL, false, 100);
    CHECK(fault_active(&m, FAULT_FAN_STALL));             /* latched */
    fault_clear(&m, FAULT_FAN_STALL);
    CHECK_EQ(fault_bits(&m), 0);
}
void suite_fault(void) { RUN(fault_qualify); }

static void time_roundtrip(void)
{
    datetime_t d = { 2026, 10, 3, 7, 30, 15, 0 };
    uint32_t t = datetime_to_unix(&d);
    CHECK_EQ(t, 1791012615UL);                    /* date -u -d '2026-10-03 07:30:15' +%s */
    datetime_t back; unix_to_datetime(t, &back);
    CHECK_EQ(back.year, 2026); CHECK_EQ(back.month, 10); CHECK_EQ(back.day, 3);
    CHECK_EQ(back.hour, 7); CHECK_EQ(back.minute, 30); CHECK_EQ(back.second, 15);
    CHECK_EQ(back.weekday, 6);                    /* Saturday */
    datetime_t leap = { 2028, 2, 29, 23, 59, 59, 0 };
    unix_to_datetime(datetime_to_unix(&leap), &back);
    CHECK_EQ(back.month, 2); CHECK_EQ(back.day, 29);
}
void suite_timeconv(void) { RUN(time_roundtrip); }

static char cli_out[512];
static void cap_write(const char *s) { strncat(cli_out, s, sizeof cli_out - strlen(cli_out) - 1); }
static int  last_argc; static char last_arg1[16];
static int  h_set(cli_t *c, int argc, char *argv[]) { (void)c; last_argc = argc;
    strncpy(last_arg1, argc > 1 ? argv[1] : "", 15); return 0; }
static void cli_dispatch(void)
{
    static const cli_cmd_t cmds[] = { { "duty", "set duty", h_set } };
    cli_t c; cli_init(&c, cmds, 1, cap_write, false);
    const char *in = "  duty   75 \r";
    for (const char *p = in; *p; p++) cli_input(&c, *p);
    CHECK_EQ(last_argc, 2); CHECK(strcmp(last_arg1, "75") == 0);
    cli_out[0] = 0;
    for (const char *p = "dutz\by 1\r"; *p; p++) cli_input(&c, *p);  /* backspace edits */
    CHECK(strstr(cli_out, "unknown") == NULL);
    CHECK_EQ(last_argc, 1 + 1);
    cli_out[0] = 0;
    char line[] = "foo";
    CHECK_EQ(cli_execute(&c, line), -1);
    CHECK(strstr(cli_out, "unknown command") != NULL);
}
void suite_cli(void) { RUN(cli_dispatch); }
