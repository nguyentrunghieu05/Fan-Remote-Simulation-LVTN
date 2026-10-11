/**
 * @file    bme280.h
 * @brief   Bosch BME280 / BMP280 temperature, pressure (and humidity) over I2C.
 *
 * The chip is told apart by its ID register (0xD0): 0x60 = BME280 (has a
 * humidity sensor), 0x58 = BMP280 (no humidity). Measurements use forced
 * mode: write ctrl_meas -> wait ~10 ms -> burst read 0xF7..0xFE. The raw
 * values are turned into units with the integer formulas of the Bosch
 * datasheet and the factory calibration read once at init.
 */
#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

#define BME280_ADDR_DEFAULT  0x76U   /* SDO low  */
#define BME280_ADDR_ALT      0x77U   /* SDO high */
#define BME280_CHIP_ID       0x60U
#define BMP280_CHIP_ID       0x58U

typedef struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
} bme280_calib_t;

typedef struct {
    i2c_bus_t      bus;
    delay_ms_fn    delay_ms;
    uint8_t        addr;
    uint8_t        chip_id;
    bool           has_humidity;
    bme280_calib_t cal;
} bme280_t;

typedef struct {
    int16_t  temp_c100;   /* 0.01 °C                      */
    uint16_t hum_c100;    /* 0.01 %RH, 0 on a BMP280       */
    uint32_t press_pa;    /* Pa                           */
} bme280_sample_t;

/** Probe addr, read the ID and the calibration. ERR_NOT_FOUND if absent / unknown ID. */
int  bme280_init(bme280_t *dev, const i2c_bus_t *bus, uint8_t addr, delay_ms_fn delay_ms);
/** One forced-mode measurement. */
int  bme280_measure(bme280_t *dev, bme280_sample_t *out);
/** Datasheet compensation, exposed for unit tests. adc_h is ignored without humidity. */
void bme280_compensate(const bme280_t *dev, int32_t adc_t, int32_t adc_p, int32_t adc_h, bme280_sample_t *out);

#endif /* BME280_H */
