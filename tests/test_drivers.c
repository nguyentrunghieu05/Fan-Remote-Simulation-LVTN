/* Driver tests against simulated chips: the driver code is byte-for-byte
 * the one that runs on the STM32, only the bus functions are fakes. */
#include "test.h"
#include <string.h>
#include <stdlib.h>
#include "w25qxx.h"
#include "sht3x.h"
#include "ssd1306.h"
#include "crc.h"
#include "err.h"

static void no_delay(uint32_t ms) { (void)ms; }

/* ---- W25Q64 simulator: decodes the SPI command stream ---- */
typedef struct {
    uint8_t  mem[1 << 16];      /* only 64 KB simulated, ID still says 8 MB */
    bool     cs, wel;
    uint8_t  cmd; uint32_t addr; int phase; int busy_polls;
} w25sim_t;
static w25sim_t sim;

static void sim_select(void *ctx, bool on)
{
    (void)ctx;
    if (on) { sim.phase = 0; }
    else if (sim.cmd == 0x02 || sim.cmd == 0x20) { sim.wel = false; sim.busy_polls = 2; }
    sim.cs = on;
}
static uint8_t sim_byte(uint8_t in)
{
    uint8_t out = 0xFF;
    if (sim.phase == 0) { sim.cmd = in; sim.addr = 0;
        if (in == 0x06) sim.wel = true;
    } else if ((sim.cmd == 0x03 || sim.cmd == 0x02 || sim.cmd == 0x20) && sim.phase <= 3) {
        sim.addr = (sim.addr << 8) | in;
        if (sim.phase == 3 && sim.cmd == 0x20 && sim.wel) {
            uint32_t a = (sim.addr & 0xFFFFu) & ~0xFFFu; memset(&sim.mem[a], 0xFF, 4096);
        }
    } else switch (sim.cmd) {
        case 0x9F: { static const uint8_t id[3] = { 0xEF, 0x40, 0x17 }; out = id[(sim.phase - 1) % 3]; break; }
        case 0x05: out = (uint8_t)((sim.busy_polls > 0 ? 1 : 0) | (sim.wel ? 2 : 0));
                   if (sim.busy_polls > 0) { sim.busy_polls--; }
                   break;
        case 0x03: out = sim.mem[sim.addr & 0xFFFFu]; sim.addr++; break;
        case 0x02: if (sim.wel) { uint32_t page = sim.addr & ~0xFFu;
                       sim.mem[(page | (sim.addr & 0xFF)) & 0xFFFFu] &= in;   /* wraps in page */
                       sim.addr = page | ((sim.addr + 1) & 0xFF); } break;
        default: break;
    }
    sim.phase++;
    return out;
}
static int sim_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx;
    if (!sim.cs) return ERR_IO;
    for (size_t i = 0; i < n; i++) { uint8_t o = sim_byte(tx ? tx[i] : 0xFF); if (rx) rx[i] = o; }
    return ERR_OK;
}

static void w25_id_rw(void)
{
    memset(&sim, 0, sizeof sim); memset(sim.mem, 0xFF, sizeof sim.mem);
    spi_bus_t bus = { sim_xfer, sim_select, NULL };
    w25qxx_t d;
    CHECK_EQ(w25qxx_init(&d, &bus, no_delay), ERR_OK);
    CHECK_EQ(d.jedec_id, 0xEF4017); CHECK_EQ(d.size, 8u * 1024 * 1024);

    uint8_t data[600], back[600];
    for (int i = 0; i < 600; i++) data[i] = (uint8_t)(i * 7);
    CHECK_EQ(w25qxx_write(&d, 0x1F0, data, sizeof data), ERR_OK);   /* crosses 3 pages */
    CHECK_EQ(w25qxx_read(&d, 0x1F0, back, sizeof back), ERR_OK);
    CHECK(memcmp(data, back, sizeof data) == 0);
    CHECK_EQ(w25qxx_erase_sector(&d, 0x0123), ERR_OK);
    CHECK_EQ(w25qxx_read(&d, 0x1F0, back, 16), ERR_OK);
    CHECK_EQ(back[0], 0xFF);
    CHECK_EQ(w25qxx_read(&d, d.size - 1, back, 2), ERR_PARAM);
}
static int dead_xfer(void *c, const uint8_t *t, uint8_t *r, size_t n)
{ (void)c; (void)t; if (r) memset(r, 0xFF, n); return ERR_OK; }
static void dead_sel(void *c, bool on) { (void)c; (void)on; }
static void w25_missing(void)
{
    spi_bus_t bus = { dead_xfer, dead_sel, NULL };   /* MISO pulled high, no chip */
    w25qxx_t d;
    CHECK_EQ(w25qxx_init(&d, &bus, no_delay), ERR_NOT_FOUND);
}
void suite_w25qxx(void) { RUN(w25_id_rw); RUN(w25_missing); }

/* ---- SHT3x fake ---- */
static uint8_t sht_reply[6]; static int sht_nack; static uint16_t sht_last_cmd;
static int sh_write(void *c, uint8_t a, const uint8_t *d, size_t n)
{ (void)c; if (sht_nack || a != 0x44 || n != 2) return ERR_IO; sht_last_cmd = (uint16_t)(d[0] << 8 | d[1]); return ERR_OK; }
static int sh_read(void *c, uint8_t a, uint8_t *d, size_t n)
{ (void)c; (void)a; memcpy(d, sht_reply, n); return ERR_OK; }
static void set_reply(uint16_t t, uint16_t h)
{
    sht_reply[0] = (uint8_t)(t >> 8); sht_reply[1] = (uint8_t)t; sht_reply[2] = crc8_sensirion(sht_reply, 2);
    sht_reply[3] = (uint8_t)(h >> 8); sht_reply[4] = (uint8_t)h; sht_reply[5] = crc8_sensirion(&sht_reply[3], 2);
}
static void sht_measure(void)
{
    i2c_bus_t bus = { sh_write, sh_read, NULL };
    sht3x_t d; sht3x_sample_t s;
    sht_nack = 0;
    CHECK_EQ(sht3x_init(&d, &bus, SHT3X_ADDR_DEFAULT, no_delay), ERR_OK);
    CHECK_EQ(sht_last_cmd, 0x30A2);
    set_reply(0x6666, 0x8000);         /* 25.0 °C, 50.0 %RH */
    CHECK_EQ(sht3x_measure(&d, &s), ERR_OK);
    CHECK_EQ(sht_last_cmd, 0x2400);
    CHECK_EQ(s.temp_c100, 2500); CHECK_EQ(s.hum_c100, 5000);
    sht3x_convert(0, 0xFFFF, &s);  CHECK_EQ(s.temp_c100, -4500); CHECK_EQ(s.hum_c100, 10000);
    sht3x_convert(0xFFFF, 0, &s);  CHECK_EQ(s.temp_c100, 13000);
    sht_reply[5] ^= 1;
    CHECK_EQ(sht3x_measure(&d, &s), ERR_CRC);
    sht_nack = 1;
    CHECK_EQ(sht3x_measure(&d, &s), ERR_IO);
}
static int oled_writes; static size_t oled_bytes;
static int ol_write(void *c, uint8_t a, const uint8_t *d, size_t n)
{ (void)c; (void)d; if (a != 0x3C) return ERR_IO; oled_writes++; oled_bytes += n; return ERR_OK; }
static void oled_draw(void)
{
    i2c_bus_t bus = { ol_write, sh_read, NULL };
    static ssd1306_t o;
    CHECK_EQ(ssd1306_init(&o, &bus, SSD1306_ADDR), ERR_OK);
    ssd1306_text(&o, 0, 0, "A");
    CHECK_EQ(o.fb[0], 0x7E); CHECK_EQ(o.fb[2], 0x11);      /* glyph of 'A' */
    ssd1306_bar(&o, 0, 16, 100, 8, 50);
    CHECK(o.fb[2 * 128 + 10] != 0); CHECK_EQ(o.fb[2 * 128 + 80] & 0x3C, 0);  /* half filled */
    oled_writes = 0; oled_bytes = 0;
    CHECK_EQ(ssd1306_flush(&o), ERR_OK);
    CHECK_EQ(oled_writes, 9); CHECK_EQ(oled_bytes, 7 + 8 * 129);
}
void suite_sht3x(void) { RUN(sht_measure); RUN(oled_draw); }
