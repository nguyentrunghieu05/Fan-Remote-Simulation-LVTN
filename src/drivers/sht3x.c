#include "sht3x.h"
#include "crc.h"
#include "err.h"

static int send_cmd(sht3x_t *d, uint16_t cmd)
{
    uint8_t b[2] = { (uint8_t)(cmd >> 8), (uint8_t)cmd };
    return d->bus.write(d->bus.ctx, d->addr, b, 2U);
}

int sht3x_soft_reset(sht3x_t *dev)
{
    int rc = send_cmd(dev, 0x30A2U);
    dev->delay_ms(2U);
    return rc;
}

int sht3x_init(sht3x_t *dev, const i2c_bus_t *bus, uint8_t addr, delay_ms_fn delay_ms)
{
    dev->bus      = *bus;
    dev->addr     = addr;
    dev->delay_ms = delay_ms;
    return sht3x_soft_reset(dev);           /* NACK here = sensor missing */
}

void sht3x_convert(uint16_t raw_t, uint16_t raw_rh, sht3x_sample_t *out)
{
    /* Datasheet: T = -45 + 175 * raw / 65535 ; RH = 100 * raw / 65535.
     * Done in integers, scaled by 100, rounded. */
    int32_t t = -4500 + (int32_t)(((int64_t)17500 * raw_t + 32767) / 65535);
    int32_t h = (int32_t)(((int64_t)10000 * raw_rh + 32767) / 65535);
    out->temp_c100 = (int16_t)t;
    out->hum_c100  = (uint16_t)h;
}

int sht3x_measure(sht3x_t *dev, sht3x_sample_t *out)
{
    int rc = send_cmd(dev, 0x2400U);        /* single shot, high rep., no stretch */
    if (rc != ERR_OK) {
        return rc;
    }
    dev->delay_ms(16U);                     /* tMEAS,h = 15.5 ms max */

    uint8_t b[6];
    rc = dev->bus.read(dev->bus.ctx, dev->addr, b, sizeof b);
    if (rc != ERR_OK) {
        return rc;
    }
    if (crc8_sensirion(&b[0], 2U) != b[2] || crc8_sensirion(&b[3], 2U) != b[5]) {
        return ERR_CRC;
    }
    sht3x_convert((uint16_t)((b[0] << 8) | b[1]), (uint16_t)((b[3] << 8) | b[4]), out);
    return ERR_OK;
}
