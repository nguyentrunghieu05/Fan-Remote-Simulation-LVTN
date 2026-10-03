#include "w25qxx.h"
#include "err.h"

/* Command set (datasheet section 8) */
#define CMD_WRITE_ENABLE     0x06U
#define CMD_READ_STATUS1     0x05U
#define CMD_READ_DATA        0x03U
#define CMD_PAGE_PROGRAM     0x02U
#define CMD_SECTOR_ERASE     0x20U
#define CMD_CHIP_ERASE       0xC7U
#define CMD_JEDEC_ID         0x9FU
#define CMD_POWER_DOWN       0xB9U
#define CMD_RELEASE_PD       0xABU

#define SR1_BUSY             0x01U
#define SR1_WEL              0x02U

/* Worst-case times from the datasheet, with margin. */
#define T_PAGE_PROG_MS       5U
#define T_SECTOR_ERASE_MS    500U
#define T_CHIP_ERASE_MS      200000U

static int xfer(w25qxx_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    return d->bus.transfer(d->bus.ctx, tx, rx, len);
}

/* One command with an optional 24-bit address and an optional data phase,
 * all inside a single CS-low window. */
static int command(w25qxx_t *d, uint8_t cmd, const uint32_t *addr,
                   const uint8_t *tx, uint8_t *rx, size_t len)
{
    uint8_t hdr[4] = { cmd, 0, 0, 0 };
    size_t  hlen = 1U;
    if (addr != NULL) {
        hdr[1] = (uint8_t)(*addr >> 16);
        hdr[2] = (uint8_t)(*addr >> 8);
        hdr[3] = (uint8_t)(*addr);
        hlen = 4U;
    }
    d->bus.select(d->bus.ctx, true);
    int rc = xfer(d, hdr, NULL, hlen);
    if (rc == ERR_OK && len > 0U) {
        rc = xfer(d, tx, rx, len);
    }
    d->bus.select(d->bus.ctx, false);
    return rc;
}

static int read_status(w25qxx_t *d, uint8_t *sr)
{
    return command(d, CMD_READ_STATUS1, NULL, NULL, sr, 1U);
}

static int wait_ready(w25qxx_t *d, uint32_t timeout_ms)
{
    uint8_t  sr  = SR1_BUSY;
    uint32_t elapsed = 0U;
    for (;;) {
        int rc = read_status(d, &sr);
        if (rc != ERR_OK) {
            return rc;
        }
        if ((sr & SR1_BUSY) == 0U) {
            return ERR_OK;
        }
        if (elapsed >= timeout_ms) {
            return ERR_TIMEOUT;
        }
        d->delay_ms(1U);
        elapsed++;
    }
}

static int write_enable(w25qxx_t *d)
{
    int rc = command(d, CMD_WRITE_ENABLE, NULL, NULL, NULL, 0U);
    uint8_t sr = 0U;
    if (rc == ERR_OK) {
        rc = read_status(d, &sr);
    }
    if (rc == ERR_OK && (sr & SR1_WEL) == 0U) {
        rc = ERR_IO;                        /* write protected or not answering */
    }
    return rc;
}

int w25qxx_read_jedec(w25qxx_t *dev, uint32_t *id)
{
    uint8_t b[3] = { 0 };
    int rc = command(dev, CMD_JEDEC_ID, NULL, NULL, b, 3U);
    *id = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return rc;
}

int w25qxx_init(w25qxx_t *dev, const spi_bus_t *bus, delay_ms_fn delay_ms)
{
    dev->bus      = *bus;
    dev->delay_ms = delay_ms;
    dev->size     = 0U;

    (void)command(dev, CMD_RELEASE_PD, NULL, NULL, NULL, 0U);
    dev->delay_ms(1U);                      /* tRES1 = 3 us */

    int rc = w25qxx_read_jedec(dev, &dev->jedec_id);
    if (rc != ERR_OK) {
        return rc;
    }
    uint8_t manuf = (uint8_t)(dev->jedec_id >> 16);
    uint8_t cap   = (uint8_t)(dev->jedec_id);
    if (manuf == 0x00U || manuf == 0xFFU || cap < 0x11U || cap > 0x19U) {
        return ERR_NOT_FOUND;               /* bus floating or unknown part */
    }
    dev->size = 1UL << cap;                 /* 0x17 -> 8 MB (W25Q64) */
    return ERR_OK;
}

int w25qxx_read(w25qxx_t *dev, uint32_t addr, void *buf, size_t len)
{
    if (addr + len > dev->size) {
        return ERR_PARAM;
    }
    return command(dev, CMD_READ_DATA, &addr, NULL, (uint8_t *)buf, len);
}

int w25qxx_write(w25qxx_t *dev, uint32_t addr, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    if (addr + len > dev->size) {
        return ERR_PARAM;
    }
    while (len > 0U) {
        /* A page program wraps inside the page, so never cross a boundary. */
        size_t room  = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
        size_t chunk = (len < room) ? len : room;
        int rc = write_enable(dev);
        if (rc == ERR_OK) {
            rc = command(dev, CMD_PAGE_PROGRAM, &addr, p, NULL, chunk);
        }
        if (rc == ERR_OK) {
            rc = wait_ready(dev, T_PAGE_PROG_MS);
        }
        if (rc != ERR_OK) {
            return rc;
        }
        addr += (uint32_t)chunk;
        p    += chunk;
        len  -= chunk;
    }
    return ERR_OK;
}

int w25qxx_erase_sector(w25qxx_t *dev, uint32_t addr)
{
    addr -= addr % W25Q_SECTOR_SIZE;
    if (addr >= dev->size) {
        return ERR_PARAM;
    }
    int rc = write_enable(dev);
    if (rc == ERR_OK) {
        rc = command(dev, CMD_SECTOR_ERASE, &addr, NULL, NULL, 0U);
    }
    return (rc == ERR_OK) ? wait_ready(dev, T_SECTOR_ERASE_MS) : rc;
}

int w25qxx_erase_chip(w25qxx_t *dev)
{
    int rc = write_enable(dev);
    if (rc == ERR_OK) {
        rc = command(dev, CMD_CHIP_ERASE, NULL, NULL, NULL, 0U);
    }
    return (rc == ERR_OK) ? wait_ready(dev, T_CHIP_ERASE_MS) : rc;
}

int w25qxx_power_down(w25qxx_t *dev)
{
    return command(dev, CMD_POWER_DOWN, NULL, NULL, NULL, 0U);
}

/* ---- flash_dev_t adapter ------------------------------------------------ */
static int fd_read(void *ctx, uint32_t a, void *b, size_t n)        { return w25qxx_read((w25qxx_t *)ctx, a, b, n); }
static int fd_write(void *ctx, uint32_t a, const void *b, size_t n) { return w25qxx_write((w25qxx_t *)ctx, a, b, n); }
static int fd_erase(void *ctx, uint32_t a)                          { return w25qxx_erase_sector((w25qxx_t *)ctx, a); }

void w25qxx_as_flash(w25qxx_t *dev, flash_dev_t *out)
{
    out->read         = fd_read;
    out->write        = fd_write;
    out->erase_sector = fd_erase;
    out->size         = dev->size;
    out->sector_size  = W25Q_SECTOR_SIZE;
    out->ctx          = dev;
}
