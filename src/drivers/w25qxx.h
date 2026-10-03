/**
 * @file    w25qxx.h
 * @brief   Winbond W25Qxx SPI NOR flash driver (W25Q16 / 32 / 64 / 128).
 *
 * SPI mode 0, up to 50 MHz for the read command 0x03. Organisation:
 * 256-byte pages (program unit), 4 KB sectors (erase unit).
 * The driver only uses spi_bus_t, so it runs on any MCU, and in the unit
 * tests against a simulated chip.
 */
#ifndef W25QXX_H
#define W25QXX_H

#include <stdint.h>
#include "bus.h"

#define W25Q_PAGE_SIZE     256U
#define W25Q_SECTOR_SIZE   4096U

typedef struct {
    spi_bus_t   bus;
    delay_ms_fn delay_ms;
    uint32_t    jedec_id;   /* e.g. 0xEF4017 = W25Q64 */
    uint32_t    size;       /* bytes, from the JEDEC capacity code */
} w25qxx_t;

/** Wake the chip, read and check the JEDEC ID. ERR_NOT_FOUND if no flash answers. */
int w25qxx_init(w25qxx_t *dev, const spi_bus_t *bus, delay_ms_fn delay_ms);
int w25qxx_read_jedec(w25qxx_t *dev, uint32_t *id);
int w25qxx_read(w25qxx_t *dev, uint32_t addr, void *buf, size_t len);
/** Program any length; split on page boundaries internally. */
int w25qxx_write(w25qxx_t *dev, uint32_t addr, const void *buf, size_t len);
int w25qxx_erase_sector(w25qxx_t *dev, uint32_t addr);
int w25qxx_erase_chip(w25qxx_t *dev);
int w25qxx_power_down(w25qxx_t *dev);

/** Fill a generic flash_dev_t so storage services can use this chip. */
void w25qxx_as_flash(w25qxx_t *dev, flash_dev_t *out);

#endif /* W25QXX_H */
