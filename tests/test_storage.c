#include "test.h"
#include <string.h>
#include "modbus_slave.h"
#include "crc.h"
#include "datalog.h"
#include "config_store.h"
#include "telemetry.h"
#include "err.h"
#include "fake_flash.h"

/* ---------------- Modbus ---------------- */
static uint16_t regs[16];
static mb_exception_t rd(void *ctx, mb_table_t t, uint16_t a, uint16_t *v)
{ (void)ctx; if (a >= 16) return MB_EX_ILLEGAL_ADDRESS; *v = (uint16_t)(t == MB_TABLE_INPUT ? 1000 + a : regs[a]); return MB_EX_NONE; }
static mb_exception_t wr(void *ctx, uint16_t a, uint16_t v)
{ (void)ctx; if (a >= 16) return MB_EX_ILLEGAL_ADDRESS; if (v > 1000) return MB_EX_ILLEGAL_VALUE; regs[a] = v; return MB_EX_NONE; }
static const mb_callbacks_t cbs = { rd, wr, NULL };

static size_t frame(uint8_t *f, size_t n)    /* append CRC */
{ uint16_t c = crc16_modbus(f, n); f[n] = (uint8_t)c; f[n + 1] = (uint8_t)(c >> 8); return n + 2; }

static void mb_read_write(void)
{
    mb_slave_t s; mb_slave_init(&s, 7, &cbs);
    uint8_t req[32], rsp[MB_FRAME_MAX];

    /* FC04 read 2 input registers at 3 */
    uint8_t r1[] = { 7, 0x04, 0, 3, 0, 2 }; memcpy(req, r1, 6);
    size_t n = mb_slave_process(&s, req, frame(req, 6), rsp);
    CHECK_EQ(n, 9); CHECK_EQ(rsp[2], 4); CHECK_EQ((rsp[3] << 8) | rsp[4], 1003);
    CHECK_EQ(crc16_modbus(rsp, n - 2), (uint16_t)(rsp[n - 2] | (rsp[n - 1] << 8)));

    /* FC06 write 500 to holding 2, then FC03 read back */
    uint8_t r2[] = { 7, 0x06, 0, 2, 0x01, 0xF4 }; memcpy(req, r2, 6);
    n = mb_slave_process(&s, req, frame(req, 6), rsp);
    CHECK_EQ(n, 8); CHECK_EQ(regs[2], 500); CHECK(memcmp(rsp, req, 8) == 0);   /* echo */

    /* FC10 write 2 regs at 4 */
    uint8_t r3[] = { 7, 0x10, 0, 4, 0, 2, 4, 0, 10, 0, 20 }; memcpy(req, r3, 11);
    n = mb_slave_process(&s, req, frame(req, 11), rsp);
    CHECK_EQ(n, 8); CHECK_EQ(regs[4], 10); CHECK_EQ(regs[5], 20);
}
static void mb_errors(void)
{
    mb_slave_t s; mb_slave_init(&s, 7, &cbs);
    uint8_t req[32], rsp[MB_FRAME_MAX];
    uint8_t bad_addr[] = { 7, 0x03, 0, 15, 0, 2 }; memcpy(req, bad_addr, 6);
    size_t n = mb_slave_process(&s, req, frame(req, 6), rsp);
    CHECK_EQ(n, 5); CHECK_EQ(rsp[1], 0x83); CHECK_EQ(rsp[2], MB_EX_ILLEGAL_ADDRESS);

    uint8_t bad_fc[] = { 7, 0x05, 0, 0, 0xFF, 0 }; memcpy(req, bad_fc, 6);
    n = mb_slave_process(&s, req, frame(req, 6), rsp);
    CHECK_EQ(rsp[1], 0x85); CHECK_EQ(rsp[2], MB_EX_ILLEGAL_FUNCTION);

    uint8_t bad_val[] = { 7, 0x06, 0, 1, 0x27, 0x10 }; memcpy(req, bad_val, 6);   /* 10000 */
    n = mb_slave_process(&s, req, frame(req, 6), rsp);
    CHECK_EQ(rsp[2], MB_EX_ILLEGAL_VALUE);

    uint8_t other[] = { 9, 0x03, 0, 0, 0, 1 }; memcpy(req, other, 6);
    CHECK_EQ(mb_slave_process(&s, req, frame(req, 6), rsp), 0);   /* other slave */

    memcpy(req, bad_addr, 6); n = frame(req, 6); req[n - 1] ^= 0x55;
    CHECK_EQ(mb_slave_process(&s, req, n, rsp), 0);              /* bad CRC: silent */
    CHECK_EQ(s.crc_errors, 1);

    uint8_t bc[] = { 0, 0x06, 0, 3, 0, 42 }; memcpy(req, bc, 6);
    CHECK_EQ(mb_slave_process(&s, req, frame(req, 6), rsp), 0);  /* broadcast: no reply */
    CHECK_EQ(regs[3], 42);                                        /* ...but executed */
}
void suite_modbus(void) { RUN(mb_read_write); RUN(mb_errors); }

/* ---------------- Data log ---------------- */
#define SEC 4096u
static datalog_sample_t sample(uint32_t i)
{
    datalog_sample_t s = { .timestamp = 1000 + i, .temp_c100 = (int16_t)(2500 + i),
                           .hum_c100 = 5000, .rpm = 1200, .duty_c10 = 500, .state = 2, .faults = 0 };
    return s;
}
static void log_append_reboot(void)
{
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 16 * SEC, SEC);
    datalog_t log;
    CHECK_EQ(datalog_init(&log, &fd, 2 * SEC, 4), ERR_OK);
    CHECK_EQ(datalog_count(&log), 0);
    for (uint32_t i = 0; i < 200; i++) { datalog_sample_t s = sample(i); CHECK_EQ(datalog_append(&log, &s), ERR_OK); }
    CHECK_EQ(datalog_count(&log), 200);

    datalog_t log2;                                      /* "reboot" */
    CHECK_EQ(datalog_init(&log2, &fd, 2 * SEC, 4), ERR_OK);
    CHECK_EQ(datalog_count(&log2), 200);
    CHECK_EQ(log2.next_seq, 201);
    datalog_sample_t s; uint32_t seq;
    CHECK_EQ(datalog_read(&log2, 0, &s, &seq), ERR_OK); CHECK_EQ(seq, 1); CHECK_EQ(s.timestamp, 1000);
    CHECK_EQ(datalog_read(&log2, 199, &s, &seq), ERR_OK); CHECK_EQ(seq, 200);
    CHECK(ff.mem[0] == 0xFF && ff.mem[6 * SEC] == 0xFF);  /* stays inside its area */
    fake_flash_free(&ff);
}
static void log_wraparound(void)
{
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 4 * SEC, SEC);
    datalog_t log; datalog_init(&log, &fd, 0, 4);       /* 4 x 128 = 512 slots */
    for (uint32_t i = 0; i < 1300; i++) { datalog_sample_t s = sample(i); datalog_append(&log, &s); }
    /* 1300 = 10 sectors + 20: ring holds 3 full sectors + 20 records */
    CHECK_EQ(datalog_count(&log), 3 * 128 + 20);
    datalog_sample_t s; uint32_t seq;
    datalog_read(&log, 0, &s, &seq);
    CHECK_EQ(seq, 1300 - (3 * 128 + 20) + 1);            /* oldest kept */
    datalog_read(&log, datalog_count(&log) - 1, &s, &seq);
    CHECK_EQ(seq, 1300);

    datalog_t log2; datalog_init(&log2, &fd, 0, 4);      /* reboot after wrap */
    CHECK_EQ(datalog_count(&log2), datalog_count(&log));
    CHECK_EQ(log2.next_seq, 1301);
    datalog_read(&log2, 0, &s, &seq); CHECK_EQ(seq, 1300 - (3 * 128 + 20) + 1);

    /* exactly full ring then reboot */
    datalog_format(&log);
    for (uint32_t i = 0; i < 512; i++) { datalog_sample_t x = sample(i); datalog_append(&log, &x); }
    datalog_init(&log2, &fd, 0, 4);
    CHECK_EQ(datalog_count(&log2), 512);
    CHECK_EQ(log2.next_seq, 513);
    fake_flash_free(&ff);
}
static void log_power_cut_and_upload(void)
{
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 4 * SEC, SEC);
    datalog_t log; datalog_init(&log, &fd, 0, 4);
    for (uint32_t i = 0; i < 10; i++) { datalog_sample_t s = sample(i); datalog_append(&log, &s); }
    ff.cut_after = 12;                                   /* power dies mid-record */
    datalog_sample_t s = sample(10);
    CHECK(datalog_append(&log, &s) != ERR_OK);
    ff.cut_after = -1;

    datalog_t log2; datalog_init(&log2, &fd, 0, 4);
    CHECK_EQ(datalog_count(&log2), 11);                  /* the torn slot is counted... */
    uint32_t seq;
    CHECK_EQ(datalog_read(&log2, 10, &s, &seq), ERR_CRC);/* ...but detected as damaged */
    CHECK_EQ(log2.next_seq, 11);
    s = sample(11); CHECK_EQ(datalog_append(&log2, &s), ERR_OK);

    /* upload: mark first 3 as sent, reboot, next unsent must be #4 */
    uint32_t slot;
    for (int i = 0; i < 3; i++) {
        CHECK_EQ(datalog_next_unsent(&log2, &slot, &seq, &s), ERR_OK);
        CHECK_EQ(seq, (uint32_t)i + 1);
        CHECK_EQ(datalog_mark_sent(&log2, slot), ERR_OK);
    }
    datalog_t log3; datalog_init(&log3, &fd, 0, 4);
    CHECK_EQ(datalog_next_unsent(&log3, &slot, &seq, &s), ERR_OK);
    CHECK_EQ(seq, 4);
    CHECK_EQ(datalog_read(&log3, 0, &s, &seq), ERR_OK);  /* sent records still readable */
    int left = 0;
    while (datalog_next_unsent(&log3, &slot, &seq, &s) == ERR_OK) { datalog_mark_sent(&log3, slot); left++; }
    CHECK_EQ(left, 8);                                   /* #4..#10 and #11(new) */
    fake_flash_free(&ff);
}
void suite_datalog(void) { RUN(log_append_reboot); RUN(log_wraparound); RUN(log_power_cut_and_upload); }

/* ---------------- Config store ---------------- */
typedef struct { uint32_t a; float k; char name[20]; } cfg_t;
static void cfg_ab(void)
{
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 4 * SEC, SEC);
    config_store_t cs; cfg_t c = { 0 };
    config_store_init(&cs, &fd, 0, SEC);
    CHECK_EQ(config_store_load(&cs, &c, sizeof c), ERR_NOT_FOUND);
    cfg_t v1 = { 1, 1.5f, "first" }, v2 = { 2, 2.5f, "second" };
    CHECK_EQ(config_store_save(&cs, &v1, sizeof v1), ERR_OK);
    CHECK_EQ(config_store_save(&cs, &v2, sizeof v2), ERR_OK);
    config_store_init(&cs, &fd, 0, SEC);
    CHECK_EQ(config_store_load(&cs, &c, sizeof c), ERR_OK);
    CHECK_EQ(c.a, 2);

    ff.cut_after = 10;                                    /* power cut while saving v3 */
    cfg_t v3 = { 3, 3.5f, "third" };
    CHECK(config_store_save(&cs, &v3, sizeof v3) != ERR_OK);
    ff.cut_after = -1;
    config_store_init(&cs, &fd, 0, SEC);
    CHECK_EQ(config_store_load(&cs, &c, sizeof c), ERR_OK);
    CHECK_EQ(c.a, 2);                                     /* old copy survived */
    CHECK(strcmp(c.name, "second") == 0);
    CHECK_EQ(config_store_save(&cs, &v3, sizeof v3), ERR_OK);
    config_store_init(&cs, &fd, 0, SEC);
    config_store_load(&cs, &c, sizeof c);
    CHECK_EQ(c.a, 3);
    fake_flash_free(&ff);
}
void suite_config_store(void) { RUN(cfg_ab); }

/* ---------------- Telemetry ---------------- */
static void tlm_roundtrip(void)
{
    char line[TLM_LINE_MAX];
    datalog_sample_t s = { 1791012615UL, 2734, 5512, 1450, 625, 2, 0 };
    size_t n = tlm_encode_sample(line, sizeof line, 3, 42, &s);
    CHECK(n > 0);
    CHECK(strncmp(line, "$FAN,3,42,1791012615,2734,5512,1450,625,2,0*", 44) == 0);
    tlm_msg_t m;
    char ack[40], body[] = "ACK,3,42";
    snprintf(ack, sizeof ack, "$%s*%02X", body, tlm_checksum(body, strlen(body)));
    CHECK_EQ(tlm_parse(ack, &m), ERR_OK);
    CHECK_EQ(m.kind, TLM_ACK); CHECK_EQ(m.node, 3); CHECK_EQ(m.seq, 42);
    char cmd[40], b2[] = "CMD,3,duty,750";
    snprintf(cmd, sizeof cmd, "$%s*%02X", b2, tlm_checksum(b2, strlen(b2)));
    CHECK_EQ(tlm_parse(cmd, &m), ERR_OK);
    CHECK_EQ(m.kind, TLM_CMD); CHECK(strcmp(m.name, "duty") == 0); CHECK_EQ(m.value, 750);
    ack[strlen(ack) - 1] = (ack[strlen(ack) - 1] == '0') ? '1' : '0';
    CHECK_EQ(tlm_parse(ack, &m), ERR_CRC);
    CHECK(tlm_parse("garbage", &m) != ERR_OK);
}
void suite_telemetry(void) { RUN(tlm_roundtrip); }
