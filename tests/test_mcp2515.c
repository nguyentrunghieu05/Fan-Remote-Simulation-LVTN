/* MCP2515 CAN controller driver, run against a simulated chip that decodes
 * the SPI instructions the way the real part does (datasheet section 12). */
#include "test.h"
#include "mcp2515.h"
#include "err.h"
#include <string.h>

static void no_delay(uint32_t ms) { (void)ms; }

static struct {
    bool    present;
    bool    selected;
    uint8_t reg[128];
    /* decoding of the current CS window */
    uint8_t ins;
    uint8_t addr;
    int     pos;          /* bytes received in this window */
    uint8_t bm_mask;
    int     resets;
    int     rts;
} chip;

static void chip_reset(void)
{
    memset(chip.reg, 0, sizeof chip.reg);
    chip.reg[0x0E] = 0x80;                 /* CANSTAT: configuration mode */
    chip.reg[0x0F] = 0x87;                 /* CANCTRL reset value */
    chip.resets++;
}

static void chip_power(bool on)
{
    memset(&chip, 0, sizeof chip);
    chip.present = on;
    if (on) {
        chip_reset();
        chip.resets = 0;
    }
}

static void reg_write(uint8_t a, uint8_t v)
{
    a &= 0x7F;
    chip.reg[a] = v;
    if (a == 0x0F) {                       /* CANCTRL: mode request is applied at once */
        chip.reg[0x0E] = (uint8_t)((chip.reg[0x0E] & 0x1F) | (v & 0xE0));
    }
}

/* Loopback: TXB0 goes straight to RXB0 (or RXB1 if RXB0 is full). */
static void chip_rts(void)
{
    chip.rts++;
    if ((chip.reg[0x0E] & 0xE0) != 0x40) {
        chip.reg[0x30] |= 0x08;            /* normal mode, no bus: stays pending */
        return;
    }
    uint8_t dst = (chip.reg[0x2C] & 0x01) ? 0x71 : 0x61;
    memcpy(&chip.reg[dst], &chip.reg[0x31], 13);
    chip.reg[0x2C] |= (dst == 0x61) ? 0x01 : 0x02;
    chip.reg[0x2C] |= 0x04;                /* TX0IF */
}

static uint8_t chip_byte(uint8_t in)
{
    uint8_t out = 0xFF;
    int p = chip.pos++;
    if (p == 0) {
        chip.ins = in;
        if (in == 0xC0) { chip_reset(); }
        else if (in == 0x81) { chip_rts(); }
        else if (in == 0x40) { chip.addr = 0x31; }
        else if (in == 0x90) { chip.addr = 0x61; }
        else if (in == 0x94) { chip.addr = 0x71; }
        return out;
    }
    switch (chip.ins) {
    case 0x03:                             /* READ */
        if (p == 1) { chip.addr = in; } else { out = chip.reg[chip.addr++ & 0x7F]; }
        break;
    case 0x02:                             /* WRITE */
        if (p == 1) { chip.addr = in; } else { reg_write(chip.addr++, in); }
        break;
    case 0x05:                             /* BIT MODIFY */
        if (p == 1) { chip.addr = in; }
        else if (p == 2) { chip.bm_mask = in; }
        else if (p == 3) {
            uint8_t a = chip.addr & 0x7F;
            reg_write(a, (uint8_t)((chip.reg[a] & ~chip.bm_mask) | (in & chip.bm_mask)));
        }
        break;
    case 0x40:                             /* LOAD TX BUFFER 0 */
        reg_write(chip.addr++, in);
        break;
    case 0x90: case 0x94:                  /* READ RX BUFFER */
        out = chip.reg[chip.addr++ & 0x7F];
        break;
    case 0xA0:                             /* READ STATUS */
        out = (uint8_t)((chip.reg[0x2C] & 0x03) | ((chip.reg[0x30] & 0x08) ? 0x04 : 0));
        break;
    default:
        break;
    }
    return out;
}

static void sim_select(void *ctx, bool on)
{
    (void)ctx;
    if (on) {
        chip.pos = 0;
    } else if (chip.selected) {
        if (chip.ins == 0x90) { chip.reg[0x2C] &= (uint8_t)~0x01; }
        if (chip.ins == 0x94) { chip.reg[0x2C] &= (uint8_t)~0x02; }
    }
    chip.selected = on;
}

static int sim_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t n)
{
    (void)ctx;
    for (size_t i = 0; i < n; i++) {
        uint8_t o = chip.present ? chip_byte(tx ? tx[i] : 0xFF) : 0xFF;   /* MISO pulled up */
        if (rx) { rx[i] = o; }
    }
    return ERR_OK;
}

static const spi_bus_t k_bus = { sim_transfer, sim_select, NULL };

/* ---------------------------------------------------------------- tests */
static uint32_t timing_bitrate(uint32_t osc, const mcp2515_timing_t *t)
{
    uint32_t brp  = t->cnf1 & 0x3F;
    uint32_t prop = (t->cnf2 & 0x07) + 1, ps1 = ((t->cnf2 >> 3) & 0x07) + 1, ps2 = (t->cnf3 & 0x07) + 1;
    return osc / (2 * (brp + 1) * (1 + prop + ps1 + ps2));
}

static void mcp_timing(void)
{
    mcp2515_timing_t t;
    /* 8 MHz crystal, 500 kbit/s: 8 TQ of 250 ns, sample point 6/8 */
    CHECK_EQ(mcp2515_calc_timing(8000000, 500000, &t), ERR_OK);
    CHECK_EQ(t.cnf1, 0x00); CHECK_EQ(t.cnf2, 0x91); CHECK_EQ(t.cnf3, 0x01);
    /* 16 MHz, 500 kbit/s: 16 TQ, prop 5 + ps1 6 + ps2 4 */
    CHECK_EQ(mcp2515_calc_timing(16000000, 500000, &t), ERR_OK);
    CHECK_EQ(t.cnf1, 0x00); CHECK_EQ(t.cnf2, 0xAC); CHECK_EQ(t.cnf3, 0x03);

    static const uint32_t osc[] = { 8000000, 16000000 };
    static const uint32_t br[]  = { 125000, 250000, 500000, 1000000 };
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
            if (osc[i] == 8000000 && br[j] == 1000000) {
                CHECK_EQ(mcp2515_calc_timing(osc[i], br[j], &t), ERR_PARAM);   /* only 4 TQ */
                continue;
            }
            CHECK_EQ(mcp2515_calc_timing(osc[i], br[j], &t), ERR_OK);
            CHECK_EQ(timing_bitrate(osc[i], &t), br[j]);
            CHECK(t.cnf2 & 0x80);                              /* PS2 taken from CNF3 */
            uint32_t prop = (t.cnf2 & 7) + 1, ps1 = ((t.cnf2 >> 3) & 7) + 1, ps2 = (t.cnf3 & 7) + 1;
            CHECK(prop + ps1 >= ps2 && ps2 >= 2);              /* datasheet 5.3 rules */
        }
    }
    CHECK_EQ(mcp2515_calc_timing(8000000, 0, &t), ERR_PARAM);
    CHECK_EQ(mcp2515_calc_timing(8000000, 333333, &t), ERR_PARAM);
}

static void mcp_init(void)
{
    mcp2515_t d;
    chip_power(false);                                         /* nothing on the bus */
    CHECK_EQ(mcp2515_init(&d, &k_bus, no_delay, 8000000, 250000, MCP2515_MODE_NORMAL), ERR_NOT_FOUND);

    chip_power(true);
    CHECK_EQ(mcp2515_init(&d, &k_bus, no_delay, 8000000, 250000, MCP2515_MODE_NORMAL), ERR_OK);
    CHECK_EQ(chip.resets, 1);
    CHECK_EQ(chip.reg[0x0E] & 0xE0, 0x00);                     /* normal mode */
    CHECK_EQ(chip.reg[0x2A], 0x00);                            /* CNF1 */
    CHECK_EQ(chip.reg[0x29], 0xAC);                            /* CNF2: 16 TQ */
    CHECK_EQ(chip.reg[0x28], 0x03);                            /* CNF3 */
    CHECK_EQ(chip.reg[0x2B], 0x00);                            /* no interrupts */
    CHECK_EQ(chip.reg[0x60], 0x64);                            /* RXB0: any ID, rollover */
    CHECK_EQ(chip.reg[0x70], 0x60);
    CHECK_EQ(mcp2515_init(&d, &k_bus, no_delay, 8000000, 333333, MCP2515_MODE_NORMAL), ERR_PARAM);
}

static void mcp_loopback(void)
{
    mcp2515_t d;
    can_frame_t tx = { 0x123, 8, { 1, 2, 3, 4, 5, 6, 7, 8 } }, rx;
    chip_power(true);
    CHECK_EQ(mcp2515_init(&d, &k_bus, no_delay, 16000000, 500000, MCP2515_MODE_LOOPBACK), ERR_OK);
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_EMPTY);

    CHECK_EQ(mcp2515_send(&d, &tx), ERR_OK);
    CHECK_EQ(chip.reg[0x31], 0x24);                            /* SIDH = 0x123 >> 3 */
    CHECK_EQ(chip.reg[0x32], 0x60);                            /* SIDL = (0x123 & 7) << 5 */
    memset(&rx, 0, sizeof rx);
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_OK);
    CHECK_EQ(rx.id, 0x123); CHECK_EQ(rx.dlc, 8);
    CHECK(memcmp(rx.data, tx.data, 8) == 0);
    CHECK_EQ(chip.reg[0x2C] & 0x01, 0);                        /* RX0IF cleared by the read */
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_EMPTY);

    can_frame_t ext = { 0x18FF50E5UL | CAN_ID_EXT_FLAG, 3, { 0xAA, 0xBB, 0xCC } };
    CHECK_EQ(mcp2515_send(&d, &ext), ERR_OK);
    CHECK_EQ(chip.reg[0x32] & 0x08, 0x08);                     /* IDE bit */
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_OK);
    CHECK_EQ(rx.id, 0x18FF50E5UL | CAN_ID_EXT_FLAG); CHECK_EQ(rx.dlc, 3);
    CHECK_EQ(rx.data[2], 0xCC);

    /* two frames before reading: the second one lands in RXB1 */
    can_frame_t a = { 0x10, 1, { 0x11 } }, b = { 0x20, 1, { 0x22 } };
    CHECK_EQ(mcp2515_send(&d, &a), ERR_OK);
    CHECK_EQ(mcp2515_send(&d, &b), ERR_OK);
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_OK); CHECK_EQ(rx.id, 0x10);
    CHECK_EQ(mcp2515_receive(&d, &rx), ERR_OK); CHECK_EQ(rx.id, 0x20); CHECK_EQ(rx.data[0], 0x22);

    can_frame_t bad = { 0x1, 9, { 0 } };
    CHECK_EQ(mcp2515_send(&d, &bad), ERR_PARAM);
}

static void mcp_tx_busy_and_errors(void)
{
    mcp2515_t d;
    can_frame_t f = { 0x7FF, 2, { 9, 9 } };
    mcp2515_errors_t e;
    chip_power(true);
    CHECK_EQ(mcp2515_init(&d, &k_bus, no_delay, 8000000, 125000, MCP2515_MODE_NORMAL), ERR_OK);
    CHECK_EQ(mcp2515_send(&d, &f), ERR_OK);                    /* no ACK on an empty bus */
    CHECK_EQ(mcp2515_send(&d, &f), ERR_BUSY);                  /* TXREQ still set */
    CHECK_EQ(chip.rts, 1);
    chip.reg[0x1C] = 128; chip.reg[0x1D] = 5; chip.reg[0x2D] = 0x20;   /* error-passive */
    CHECK_EQ(mcp2515_read_errors(&d, &e), ERR_OK);
    CHECK_EQ(e.tec, 128); CHECK_EQ(e.rec, 5); CHECK_EQ(e.eflg, 0x20);
}

void suite_mcp2515(void)
{
    RUN(mcp_timing);
    RUN(mcp_init);
    RUN(mcp_loopback);
    RUN(mcp_tx_busy_and_errors);
}
