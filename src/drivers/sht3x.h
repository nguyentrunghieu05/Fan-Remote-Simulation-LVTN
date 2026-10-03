/**
 * @file    sht3x.h
 * @brief   Sensirion SHT30/31/35 temperature & humidity sensor over I2C.
 *
 * Single-shot measurement, high repeatability, no clock stretching:
 *   write 0x24 0x00 -> wait >= 15 ms -> read 6 bytes
 *   [T_msb T_lsb CRC] [RH_msb RH_lsb CRC]
 * Each 16-bit word is protected by a CRC-8 (poly 0x31), checked here.
 */
#ifndef SHT3X_H
#define SHT3X_H

#include <stdint.h>
#include "bus.h"

#define SHT3X_ADDR_DEFAULT  0x44U   /* ADDR pin low  */
#define SHT3X_ADDR_ALT      0x45U   /* ADDR pin high */

typedef struct {
    i2c_bus_t   bus;
    delay_ms_fn delay_ms;
    uint8_t     addr;
} sht3x_t;

typedef struct {
    int16_t  temp_c100;   /* 0.01 °C  */
    uint16_t hum_c100;    /* 0.01 %RH */
} sht3x_sample_t;

int sht3x_init(sht3x_t *dev, const i2c_bus_t *bus, uint8_t addr, delay_ms_fn delay_ms);
int sht3x_measure(sht3x_t *dev, sht3x_sample_t *out);
int sht3x_soft_reset(sht3x_t *dev);
/** Convert raw words (exposed for unit tests). */
void sht3x_convert(uint16_t raw_t, uint16_t raw_rh, sht3x_sample_t *out);

#endif /* SHT3X_H */
