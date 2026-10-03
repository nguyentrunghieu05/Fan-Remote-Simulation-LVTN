/**
 * @file    bus.h
 * @brief   Hardware-independent bus interfaces (SPI, I2C, flash).
 *
 * Device drivers (W25Qxx, SHT3x, SSD1306...) only talk to these structs,
 * never to the STM32 HAL. The BSP layer fills them with real functions on
 * the target; unit tests fill them with fakes on the PC. This is what lets
 * the same driver code be compiled and tested on both.
 */
#ifndef BUS_H
#define BUS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** SPI bus with one chip-select line (one struct per slave device). */
typedef struct {
    /** Full-duplex transfer. tx or rx may be NULL (sends 0xFF / discards). */
    int  (*transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    /** Drive chip select: true = selected (CS low). */
    void (*select)(void *ctx, bool active);
    void *ctx;
} spi_bus_t;

/** I2C master bus. Addresses are 7-bit (not shifted). */
typedef struct {
    int  (*write)(void *ctx, uint8_t addr, const uint8_t *data, size_t len);
    int  (*read)(void *ctx, uint8_t addr, uint8_t *data, size_t len);
    void *ctx;
} i2c_bus_t;

/** Millisecond delay supplied by the platform (vTaskDelay on target). */
typedef void (*delay_ms_fn)(uint32_t ms);

/** Generic NOR flash, used by the storage services. */
typedef struct {
    int (*read)(void *ctx, uint32_t addr, void *buf, size_t len);
    /** Program bytes; may cross page boundaries; can only clear bits (1->0). */
    int (*write)(void *ctx, uint32_t addr, const void *buf, size_t len);
    /** Erase one sector containing addr (all bytes become 0xFF). */
    int (*erase_sector)(void *ctx, uint32_t addr);
    uint32_t size;          /* total bytes */
    uint32_t sector_size;   /* erase granularity, e.g. 4096 */
    void    *ctx;
} flash_dev_t;

#endif /* BUS_H */
