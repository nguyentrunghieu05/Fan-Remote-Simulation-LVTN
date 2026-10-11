/* BME280 / BMP280 and HD44780-over-PCF8574 drivers, run against simulated
 * chips that decode the I2C traffic the way the real parts do. */
#include "test.h"
#include "bme280.h"
#include "lcd1602.h"
#include "err.h"
#include <string.h>

static void no_delay(uint32_t ms) { (void)ms; }

/* ---------------------------------------------------------------- BME280 */
/* Calibration from the BMP280 datasheet worked example (section 3.12),
 * plus humidity trimming values read from a real BME280. */
static const bme280_calib_t k_cal = {
    .T1 = 27504, .T2 = 26435, .T3 = -1000,
    .P1 = 36477, .P2 = -10685, .P3 = 3024, .P4 = 2855, .P5 = 140, .P6 = -7,
    .P7 = 15500, .P8 = -14600, .P9 = 6000,
    .H1 = 75, .H2 = 362, .H3 = 0, .H4 = 313, .H5 = 50, .H6 = 30,
};

static struct {
    uint8_t reg[256];
    uint8_t ptr;
    bool    nack;
    uint8_t addr;
    int     measurements;
} bme;

static void put16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static void bme_chip(uint8_t id, uint8_t addr)
{
    memset(&bme, 0, sizeof bme);
    bme.addr = addr;
    bme.reg[0xD0] = id;
    uint8_t *c = &bme.reg[0x88];
    put16le(&c[0], k_cal.T1); put16le(&c[2], (uint16_t)k_cal.T2); put16le(&c[4], (uint16_t)k_cal.T3);
    put16le(&c[6], k_cal.P1);
    const int16_t p[] = { k_cal.P2, k_cal.P3, k_cal.P4, k_cal.P5, k_cal.P6, k_cal.P7, k_cal.P8, k_cal.P9 };
    for (int i = 0; i < 8; i++) { put16le(&c[8 + 2 * i], (uint16_t)p[i]); }
    bme.reg[0xA1] = k_cal.H1;
    put16le(&bme.reg[0xE1], (uint16_t)k_cal.H2);
    bme.reg[0xE3] = k_cal.H3;
    /* H4 = E4[7:0] << 4 | E5[3:0],  H5 = E6[7:0] << 4 | E5[7:4] */
    bme.reg[0xE4] = (uint8_t)(k_cal.H4 >> 4);
    bme.reg[0xE5] = (uint8_t)((k_cal.H4 & 0x0F) | ((k_cal.H5 & 0x0F) << 4));
    bme.reg[0xE6] = (uint8_t)(k_cal.H5 >> 4);
    bme.reg[0xE7] = (uint8_t)k_cal.H6;
    /* raw results: press 415148, temp 519888 (20-bit, left aligned), hum 30000 */
    bme.reg[0xF7] = 0x65; bme.reg[0xF8] = 0x5A; bme.reg[0xF9] = 0xC0;
    bme.reg[0xFA] = 0x7E; bme.reg[0xFB] = 0xED; bme.reg[0xFC] = 0x00;
    bme.reg[0xFD] = 0x75; bme.reg[0xFE] = 0x30;
}

static int bme_write(void *c, uint8_t a, const uint8_t *d, size_t n)
{
    (void)c;
    if (bme.nack || a != bme.addr || n == 0) return ERR_IO;
    bme.ptr = d[0];
    for (size_t i = 1; i < n; i++) { bme.reg[(uint8_t)(bme.ptr + i - 1)] = d[i]; }
    if (n == 2 && d[0] == 0xF4 && (d[1] & 3) == 1) bme.measurements++;
    return ERR_OK;
}
static int bme_read(void *c, uint8_t a, uint8_t *d, size_t n)
{
    (void)c;
    if (bme.nack || a != bme.addr) return ERR_IO;
    for (size_t i = 0; i < n; i++) { d[i] = bme.reg[(uint8_t)(bme.ptr + i)]; }
    return ERR_OK;
}

/* Floating-point formulas of the datasheet, as an independent reference. */
static double ref_tfine(int32_t adc_t)
{
    double v1 = (adc_t / 16384.0 - k_cal.T1 / 1024.0) * k_cal.T2;
    double v2 = (adc_t / 131072.0 - k_cal.T1 / 8192.0) * (adc_t / 131072.0 - k_cal.T1 / 8192.0) * k_cal.T3;
    return v1 + v2;
}
static double ref_hum(int32_t adc_t, int32_t adc_h)
{
    double h = ref_tfine(adc_t) - 76800.0;
    h = (adc_h - (k_cal.H4 * 64.0 + k_cal.H5 / 16384.0 * h)) *
        (k_cal.H2 / 65536.0 * (1.0 + k_cal.H6 / 67108864.0 * h * (1.0 + k_cal.H3 / 67108864.0 * h)));
    h = h * (1.0 - k_cal.H1 * h / 524288.0);
    return h < 0 ? 0 : h > 100 ? 100 : h;
}

static void bme_compensation(void)
{
    bme280_t d = { .cal = k_cal, .has_humidity = true };
    bme280_sample_t s;
    bme280_compensate(&d, 519888, 415148, 30000, &s);
    CHECK_EQ(s.temp_c100, 2508);                         /* datasheet: 25.08 °C    */
    CHECK_NEAR(s.press_pa, 100653, 1);                   /* datasheet: 100653.27 Pa */
    CHECK_NEAR(s.hum_c100 / 100.0, ref_hum(519888, 30000), 0.05);
    d.has_humidity = false;
    bme280_compensate(&d, 519888, 415148, 30000, &s);
    CHECK_EQ(s.hum_c100, 0);
    /* colder air, higher pressure: monotonic behaviour */
    bme280_sample_t c;
    d.has_humidity = true;
    bme280_compensate(&d, 500000, 400000, 30000, &c);
    CHECK(c.temp_c100 < s.temp_c100);
    CHECK(c.press_pa > s.press_pa);
}

static void bme_over_i2c(void)
{
    i2c_bus_t bus = { bme_write, bme_read, NULL };
    bme280_t d;
    bme280_sample_t s;

    bme_chip(BME280_CHIP_ID, BME280_ADDR_DEFAULT);
    CHECK_EQ(bme280_init(&d, &bus, BME280_ADDR_DEFAULT, no_delay), ERR_OK);
    CHECK(d.has_humidity);
    CHECK_EQ(d.cal.T3, -1000); CHECK_EQ(d.cal.P9, 6000);
    CHECK_EQ(d.cal.H4, 313);   CHECK_EQ(d.cal.H5, 50);  CHECK_EQ(d.cal.H6, 30);   /* packed nibbles */
    CHECK_EQ(bme280_measure(&d, &s), ERR_OK);
    CHECK_EQ(bme.measurements, 1);
    CHECK_EQ(bme.reg[0xF2], 1);                          /* humidity oversampling set */
    CHECK_EQ(s.temp_c100, 2508);
    CHECK_NEAR(s.press_pa, 100653, 1);
    CHECK(s.hum_c100 > 0);

    bme_chip(BMP280_CHIP_ID, BME280_ADDR_ALT);           /* BMP280 on the other address */
    CHECK_EQ(bme280_init(&d, &bus, BME280_ADDR_DEFAULT, no_delay), ERR_IO);
    CHECK_EQ(bme280_init(&d, &bus, BME280_ADDR_ALT, no_delay), ERR_OK);
    CHECK(!d.has_humidity);
    CHECK_EQ(bme280_measure(&d, &s), ERR_OK);
    CHECK_EQ(s.hum_c100, 0); CHECK_EQ(s.temp_c100, 2508);

    bme_chip(0x61, BME280_ADDR_DEFAULT);                 /* some other chip answers */
    CHECK_EQ(bme280_init(&d, &bus, BME280_ADDR_DEFAULT, no_delay), ERR_NOT_FOUND);

    bme_chip(BME280_CHIP_ID, BME280_ADDR_DEFAULT);
    CHECK_EQ(bme280_init(&d, &bus, BME280_ADDR_DEFAULT, no_delay), ERR_OK);
    bme.reg[0xFA] = 0x80; bme.reg[0xFB] = 0; bme.reg[0xFC] = 0;   /* "skipped" marker */
    CHECK_EQ(bme280_measure(&d, &s), ERR_STATE);
    bme.nack = true;                                     /* cable pulled */
    CHECK_EQ(bme280_measure(&d, &s), ERR_IO);
}

/* ------------------------------------------------------- LCD over PCF8574 */
/* HD44780 model: watches the PCF8574 port, latches D4..D7 on EN falling. */
static struct {
    uint8_t port;
    bool    four_bit, have_high;
    int     wake_nibbles;
    uint8_t high;
    uint8_t ddram[128], addr;
    uint8_t cmds[64]; int ncmds;
    int     transactions;
    size_t  last_len;
    bool    nack;
} lcd;

static void hd_byte(uint8_t b, bool rs)
{
    if (!rs) {
        lcd.cmds[lcd.ncmds++ % 64] = b;
        if (b & 0x80) { lcd.addr = b & 0x7F; }
        else if (b == 0x01) { memset(lcd.ddram, ' ', sizeof lcd.ddram); lcd.addr = 0; }
    } else {
        lcd.ddram[lcd.addr++ & 0x7F] = b;
    }
}

static void hd_nibble(uint8_t nib, bool rs)
{
    if (!lcd.four_bit) {                 /* 8-bit mode at power-on: one nibble per command */
        lcd.wake_nibbles++;
        if (nib == 0x2) { lcd.four_bit = true; }
        return;
    }
    if (!lcd.have_high) { lcd.high = nib; lcd.have_high = true; }
    else { hd_byte((uint8_t)(lcd.high << 4 | nib), rs); lcd.have_high = false; }
}

static int lcd_write(void *c, uint8_t a, const uint8_t *d, size_t n)
{
    (void)c;
    if (lcd.nack || a != LCD1602_ADDR_DEFAULT) return ERR_IO;
    lcd.transactions++;
    lcd.last_len = n;
    for (size_t i = 0; i < n; i++) {
        if ((lcd.port & 0x04) && !(d[i] & 0x04)) {       /* EN 1 -> 0 */
            hd_nibble((uint8_t)(lcd.port >> 4), (lcd.port & 0x01) != 0);
        }
        lcd.port = d[i];
    }
    return ERR_OK;
}

static void lcd_text(void)
{
    i2c_bus_t bus = { lcd_write, bme_read, NULL };
    lcd1602_t d;
    memset(&lcd, 0, sizeof lcd);
    CHECK_EQ(lcd1602_init(&d, &bus, LCD1602_ADDR_DEFAULT, 16, 2, no_delay), ERR_OK);
    CHECK_EQ(lcd.wake_nibbles, 4);                       /* 0x3, 0x3, 0x3, 0x2 */
    CHECK(lcd.four_bit);
    CHECK_EQ(lcd.ncmds, 4);
    CHECK_EQ(lcd.cmds[0], 0x28); CHECK_EQ(lcd.cmds[1], 0x0C);
    CHECK_EQ(lcd.cmds[2], 0x06); CHECK_EQ(lcd.cmds[3], 0x01);
    CHECK(lcd.port & 0x08);                              /* backlight on */

    int t0 = lcd.transactions;
    CHECK_EQ(lcd1602_line(&d, 0, "AUTO    T 30.4C"), ERR_OK);
    CHECK_EQ(lcd.transactions - t0, 1);                  /* whole line in one I2C write */
    CHECK_EQ(lcd.last_len, 4 * 17);
    CHECK(memcmp(lcd.ddram, "AUTO    T 30.4C ", 16) == 0);   /* padded with a space */
    CHECK_EQ(lcd1602_line(&d, 1, "Out 45%"), ERR_OK);
    CHECK_EQ(lcd.cmds[(lcd.ncmds - 1) % 64], 0xC0);      /* row 1 starts at 0x40 */
    CHECK(memcmp(&lcd.ddram[0x40], "Out 45%         ", 16) == 0);
    CHECK_EQ(lcd1602_line(&d, 1, "this text is longer than 16"), ERR_OK);
    CHECK(memcmp(&lcd.ddram[0x40], "this text is lon", 16) == 0);
    CHECK_EQ(lcd.ddram[0x50], ' ');                      /* nothing past column 16 */
    CHECK_EQ(lcd1602_line(&d, 2, "x"), ERR_PARAM);       /* only 2 rows */

    CHECK_EQ(lcd1602_backlight(&d, false), ERR_OK);
    CHECK_EQ(lcd.port & 0x08, 0);

    lcd.nack = true;
    CHECK_EQ(lcd1602_init(&d, &bus, LCD1602_ADDR_DEFAULT, 16, 2, no_delay), ERR_IO);
    CHECK_EQ(lcd1602_init(&d, &bus, LCD1602_ADDR_ALT, 16, 2, no_delay), ERR_IO);
}

void suite_i2c_devices(void) { RUN(bme_compensation); RUN(bme_over_i2c); RUN(lcd_text); }
