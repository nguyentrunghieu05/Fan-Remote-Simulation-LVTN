#include "bme280.h"
#include "err.h"

#define REG_ID        0xD0U
#define REG_RESET     0xE0U
#define REG_CALIB_TP  0x88U     /* 26 bytes: T1..P9, (A0 unused), H1 at 0xA1 */
#define REG_CALIB_H   0xE1U     /* 7 bytes: H2..H6 */
#define REG_CTRL_HUM  0xF2U
#define REG_CTRL_MEAS 0xF4U
#define REG_DATA      0xF7U     /* press[3] temp[3] hum[2] */

#define OSRS_X1       1U
#define MODE_FORCED   1U

static int read_regs(bme280_t *d, uint8_t reg, uint8_t *buf, size_t n)
{
    int rc = d->bus.write(d->bus.ctx, d->addr, &reg, 1U);
    return (rc == ERR_OK) ? d->bus.read(d->bus.ctx, d->addr, buf, n) : rc;
}

static int write_reg(bme280_t *d, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return d->bus.write(d->bus.ctx, d->addr, b, 2U);
}

static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t  s16le(const uint8_t *p) { return (int16_t)u16le(p); }

int bme280_init(bme280_t *dev, const i2c_bus_t *bus, uint8_t addr, delay_ms_fn delay_ms)
{
    dev->bus      = *bus;
    dev->addr     = addr;
    dev->delay_ms = delay_ms;

    uint8_t id = 0U;
    int rc = read_regs(dev, REG_ID, &id, 1U);
    if (rc != ERR_OK) {
        return rc;                              /* NACK: nothing at this address */
    }
    if (id != BME280_CHIP_ID && id != BMP280_CHIP_ID) {
        return ERR_NOT_FOUND;                   /* another chip answers here */
    }
    dev->chip_id      = id;
    dev->has_humidity = (id == BME280_CHIP_ID);

    (void)write_reg(dev, REG_RESET, 0xB6U);     /* soft reset, then NVM copy */
    dev->delay_ms(3U);

    uint8_t c[26];
    rc = read_regs(dev, REG_CALIB_TP, c, sizeof c);
    if (rc != ERR_OK) {
        return rc;
    }
    bme280_calib_t *k = &dev->cal;
    k->T1 = u16le(&c[0]);  k->T2 = s16le(&c[2]);  k->T3 = s16le(&c[4]);
    k->P1 = u16le(&c[6]);  k->P2 = s16le(&c[8]);  k->P3 = s16le(&c[10]);
    k->P4 = s16le(&c[12]); k->P5 = s16le(&c[14]); k->P6 = s16le(&c[16]);
    k->P7 = s16le(&c[18]); k->P8 = s16le(&c[20]); k->P9 = s16le(&c[22]);
    k->H1 = c[25];

    if (dev->has_humidity) {
        uint8_t h[7];
        rc = read_regs(dev, REG_CALIB_H, h, sizeof h);
        if (rc != ERR_OK) {
            return rc;
        }
        k->H2 = s16le(&h[0]);
        k->H3 = h[2];
        k->H4 = (int16_t)(((int8_t)h[3] * 16) | (h[4] & 0x0F));
        k->H5 = (int16_t)(((int8_t)h[5] * 16) | (h[4] >> 4));
        k->H6 = (int8_t)h[6];
    }
    return ERR_OK;
}

void bme280_compensate(const bme280_t *dev, int32_t adc_t, int32_t adc_p, int32_t adc_h, bme280_sample_t *out)
{
    const bme280_calib_t *k = &dev->cal;

    /* Temperature, datasheet 4.2.3: result in 0.01 °C */
    int32_t v1 = ((((adc_t >> 3) - ((int32_t)k->T1 << 1))) * k->T2) >> 11;
    int32_t v2 = (((((adc_t >> 4) - (int32_t)k->T1) * ((adc_t >> 4) - (int32_t)k->T1)) >> 12) * k->T3) >> 14;
    int32_t t_fine = v1 + v2;
    out->temp_c100 = (int16_t)((t_fine * 5 + 128) >> 8);

    /* Pressure, 64-bit version: result in Pa * 256 */
    int64_t p1 = (int64_t)t_fine - 128000;
    int64_t p2 = p1 * p1 * k->P6;
    p2 = p2 + p1 * k->P5 * 131072;               /* << 17, written as a multiply: */
    p2 = p2 + (int64_t)k->P4 * 34359738368LL;     /* << 35  shifting a negative  */
    p1 = ((p1 * p1 * k->P3) >> 8) + p1 * k->P2 * 4096;   /* value left is UB in C */
    p1 = ((((int64_t)1) << 47) + p1) * k->P1 >> 33;
    if (p1 == 0) {
        out->press_pa = 0U;                     /* avoid division by zero */
    } else {
        int64_t p = 1048576 - adc_p;
        p = (((p << 31) - p2) * 3125) / p1;
        p1 = ((int64_t)k->P9 * (p >> 13) * (p >> 13)) >> 25;
        p2 = ((int64_t)k->P8 * p) >> 19;
        p = ((p + p1 + p2) >> 8) + (int64_t)k->P7 * 16;
        out->press_pa = (uint32_t)(p / 256);
    }

    /* Humidity: result in %RH * 1024 */
    out->hum_c100 = 0U;
    if (dev->has_humidity) {
        int32_t h = t_fine - 76800;
        h = (((((adc_h << 14) - (int32_t)k->H4 * 1048576 - ((int32_t)k->H5 * h)) + 16384) >> 15) *
             (((((((h * k->H6) >> 10) * (((h * k->H3) >> 11) + 32768)) >> 10) + 2097152) * k->H2 + 8192) >> 14));
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * k->H1) >> 4);
        h = (h < 0) ? 0 : h;
        h = (h > 419430400) ? 419430400 : h;
        out->hum_c100 = (uint16_t)(((uint32_t)h >> 12) * 100U / 1024U);
    }
}

int bme280_measure(bme280_t *dev, bme280_sample_t *out)
{
    int rc = ERR_OK;
    if (dev->has_humidity) {
        rc = write_reg(dev, REG_CTRL_HUM, OSRS_X1);   /* must be written before ctrl_meas */
    }
    if (rc == ERR_OK) {
        rc = write_reg(dev, REG_CTRL_MEAS, (uint8_t)((OSRS_X1 << 5) | (OSRS_X1 << 2) | MODE_FORCED));
    }
    if (rc != ERR_OK) {
        return rc;
    }
    dev->delay_ms(10U);                          /* 1x/1x/1x: max 9.3 ms */

    uint8_t b[8];
    rc = read_regs(dev, REG_DATA, b, dev->has_humidity ? 8U : 6U);
    if (rc != ERR_OK) {
        return rc;
    }
    int32_t adc_p = (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | (b[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)b[3] << 12) | ((uint32_t)b[4] << 4) | (b[5] >> 4));
    int32_t adc_h = dev->has_humidity ? (int32_t)((b[6] << 8) | b[7]) : 0;
    if (adc_t == 0x80000) {
        return ERR_STATE;                        /* "skipped" value: measurement did not run */
    }
    bme280_compensate(dev, adc_t, adc_p, adc_h, out);
    return ERR_OK;
}
