#include "mcp2515.h"
#include "err.h"
#include <string.h>

/* SPI instructions */
#define INS_RESET        0xC0U
#define INS_READ         0x03U
#define INS_WRITE        0x02U
#define INS_BIT_MODIFY   0x05U
#define INS_LOAD_TXB0    0x40U   /* starts at TXB0SIDH */
#define INS_RTS_TXB0     0x81U
#define INS_READ_RXB0    0x90U   /* starts at RXB0SIDH, clears RX0IF at CS high */
#define INS_READ_RXB1    0x94U
#define INS_READ_STATUS  0xA0U

/* registers */
#define REG_TEC          0x1CU
#define REG_REC          0x1DU
#define REG_CANSTAT      0x0EU
#define REG_CANCTRL      0x0FU
#define REG_CNF3         0x28U   /* CNF3, CNF2, CNF1, CANINTE follow in order */
#define REG_EFLG         0x2DU
#define REG_TXB0CTRL     0x30U
#define REG_RXB0CTRL     0x60U
#define REG_RXB1CTRL     0x70U

#define OPMOD_MASK       0xE0U
#define TXREQ            0x08U
#define RXM_ANY          0x60U   /* RXM = 11: masks / filters off */
#define BUKT             0x04U   /* RXB0 full -> roll over into RXB1 */
#define SIDL_IDE         0x08U
#define STATUS_RX0IF     0x01U
#define STATUS_RX1IF     0x02U
#define MODE_TIMEOUT_MS  10U

static int xfer(mcp2515_t *d, const uint8_t *tx, uint8_t *rx, size_t n)
{
    d->bus.select(d->bus.ctx, true);
    int rc = d->bus.transfer(d->bus.ctx, tx, rx, n);
    d->bus.select(d->bus.ctx, false);
    return rc;
}

static int read_regs(mcp2515_t *d, uint8_t reg, uint8_t *buf, size_t n)
{
    uint8_t tx[2 + 13] = { INS_READ, reg };
    uint8_t rx[2 + 13];
    if (n > 13U) {
        return ERR_PARAM;
    }
    memset(&tx[2], 0xFF, n);
    int rc = xfer(d, tx, rx, 2U + n);
    if (rc == ERR_OK) {
        memcpy(buf, &rx[2], n);
    }
    return rc;
}

static int write_regs(mcp2515_t *d, uint8_t reg, const uint8_t *buf, size_t n)
{
    uint8_t tx[2 + 13] = { INS_WRITE, reg };
    if (n > 13U) {
        return ERR_PARAM;
    }
    memcpy(&tx[2], buf, n);
    return xfer(d, tx, NULL, 2U + n);
}

static int bit_modify(mcp2515_t *d, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = { INS_BIT_MODIFY, reg, mask, val };
    return xfer(d, tx, NULL, sizeof tx);
}

int mcp2515_calc_timing(uint32_t osc_hz, uint32_t bitrate, mcp2515_timing_t *out)
{
    if (bitrate == 0U || out == NULL) {
        return ERR_PARAM;
    }
    /* Prefer many time quanta (finer sample point), 16 down to 8. */
    for (uint32_t n = 16U; n >= 8U; n--) {
        uint32_t div = 2U * bitrate * n;
        if (osc_hz % div != 0U) {
            continue;
        }
        uint32_t brp = osc_hz / div - 1U;
        if (brp > 63U) {
            continue;
        }
        uint32_t ps2  = (n + 2U) / 4U;          /* ~25 % after the sample point */
        if (ps2 < 2U) {
            ps2 = 2U;
        }
        uint32_t rest = n - 1U - ps2;           /* PropSeg + PS1 */
        uint32_t ps1  = (rest + 1U) / 2U;
        uint32_t prop = rest - ps1;
        if (ps1 < 1U || ps1 > 8U || prop < 1U || prop > 8U || ps2 > 8U) {
            continue;
        }
        out->cnf1 = (uint8_t)brp;                                   /* SJW = 1 TQ */
        out->cnf2 = (uint8_t)(0x80U | ((ps1 - 1U) << 3) | (prop - 1U));  /* BTLMODE = 1 */
        out->cnf3 = (uint8_t)(ps2 - 1U);
        return ERR_OK;
    }
    return ERR_PARAM;
}

static int set_mode(mcp2515_t *d, mcp2515_mode_t mode)
{
    int rc = bit_modify(d, REG_CANCTRL, OPMOD_MASK, (uint8_t)mode);
    for (uint32_t t = 0U; rc == ERR_OK; t++) {
        uint8_t st;
        rc = read_regs(d, REG_CANSTAT, &st, 1U);
        if (rc == ERR_OK && (st & OPMOD_MASK) == (uint8_t)mode) {
            return ERR_OK;
        }
        if (t >= MODE_TIMEOUT_MS) {
            return ERR_TIMEOUT;
        }
        d->delay_ms(1U);
    }
    return rc;
}

int mcp2515_init(mcp2515_t *dev, const spi_bus_t *bus, delay_ms_fn delay_ms,
                 uint32_t osc_hz, uint32_t bitrate, mcp2515_mode_t mode)
{
    mcp2515_timing_t tm;
    if (mcp2515_calc_timing(osc_hz, bitrate, &tm) != ERR_OK) {
        return ERR_PARAM;
    }
    dev->bus      = *bus;
    dev->delay_ms = delay_ms;
    dev->osc_hz   = osc_hz;
    dev->bitrate  = bitrate;

    uint8_t ins = INS_RESET;
    int rc = xfer(dev, &ins, NULL, 1U);
    if (rc != ERR_OK) {
        return rc;
    }
    dev->delay_ms(2U);                          /* oscillator start-up after reset */

    /* After a reset the chip is always in configuration mode. */
    uint8_t st;
    rc = read_regs(dev, REG_CANSTAT, &st, 1U);
    if (rc != ERR_OK) {
        return rc;
    }
    if ((st & OPMOD_MASK) != MCP2515_MODE_CONFIG) {
        return ERR_NOT_FOUND;
    }

    const uint8_t cnf[4] = { tm.cnf3, tm.cnf2, tm.cnf1, 0x00U };   /* CANINTE = 0: polled */
    rc = write_regs(dev, REG_CNF3, cnf, sizeof cnf);
    if (rc == ERR_OK) {
        uint8_t v = RXM_ANY | BUKT;
        rc = write_regs(dev, REG_RXB0CTRL, &v, 1U);
    }
    if (rc == ERR_OK) {
        uint8_t v = RXM_ANY;
        rc = write_regs(dev, REG_RXB1CTRL, &v, 1U);
    }
    return (rc == ERR_OK) ? set_mode(dev, mode) : rc;
}

int mcp2515_send(mcp2515_t *dev, const can_frame_t *f)
{
    if (f->dlc > CAN_MAX_DLC) {
        return ERR_PARAM;
    }
    uint8_t ctrl;
    int rc = read_regs(dev, REG_TXB0CTRL, &ctrl, 1U);
    if (rc != ERR_OK) {
        return rc;
    }
    if (ctrl & TXREQ) {
        return ERR_BUSY;
    }

    uint8_t tx[1 + 5 + CAN_MAX_DLC] = { INS_LOAD_TXB0 };
    if (f->id & CAN_ID_EXT_FLAG) {
        uint32_t id = f->id & 0x1FFFFFFFUL;
        tx[1] = (uint8_t)(id >> 21);
        tx[2] = (uint8_t)(((id >> 13) & 0xE0U) | SIDL_IDE | ((id >> 16) & 0x03U));
        tx[3] = (uint8_t)(id >> 8);
        tx[4] = (uint8_t)id;
    } else {
        uint32_t id = f->id & 0x7FFU;
        tx[1] = (uint8_t)(id >> 3);
        tx[2] = (uint8_t)((id & 0x07U) << 5);
    }
    tx[5] = f->dlc;
    memcpy(&tx[6], f->data, f->dlc);
    rc = xfer(dev, tx, NULL, 6U + f->dlc);
    if (rc == ERR_OK) {
        uint8_t rts = INS_RTS_TXB0;
        rc = xfer(dev, &rts, NULL, 1U);
    }
    return rc;
}

int mcp2515_receive(mcp2515_t *dev, can_frame_t *f)
{
    uint8_t tx[2] = { INS_READ_STATUS, 0xFF }, rx[2];
    int rc = xfer(dev, tx, rx, sizeof tx);
    if (rc != ERR_OK) {
        return rc;
    }
    uint8_t ins;
    if (rx[1] & STATUS_RX0IF) {
        ins = INS_READ_RXB0;
    } else if (rx[1] & STATUS_RX1IF) {
        ins = INS_READ_RXB1;
    } else {
        return ERR_EMPTY;
    }

    uint8_t btx[1 + 5 + CAN_MAX_DLC], b[1 + 5 + CAN_MAX_DLC];
    memset(btx, 0xFF, sizeof btx);
    btx[0] = ins;
    rc = xfer(dev, btx, b, sizeof btx);         /* CS high clears the RXnIF flag */
    if (rc != ERR_OK) {
        return rc;
    }
    uint8_t sidh = b[1], sidl = b[2];
    if (sidl & SIDL_IDE) {
        f->id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl & 0xE0U) << 13) |
                ((uint32_t)(sidl & 0x03U) << 16) | ((uint32_t)b[3] << 8) | b[4];
        f->id |= CAN_ID_EXT_FLAG;
    } else {
        f->id = ((uint32_t)sidh << 3) | (sidl >> 5);
    }
    f->dlc = b[5] & 0x0FU;
    if (f->dlc > CAN_MAX_DLC) {
        f->dlc = CAN_MAX_DLC;
    }
    memcpy(f->data, &b[6], f->dlc);
    return ERR_OK;
}

int mcp2515_read_errors(mcp2515_t *dev, mcp2515_errors_t *e)
{
    uint8_t c[2];
    int rc = read_regs(dev, REG_TEC, c, 2U);
    if (rc == ERR_OK) {
        e->tec = c[0];
        e->rec = c[1];
        rc = read_regs(dev, REG_EFLG, &e->eflg, 1U);
    }
    return rc;
}
