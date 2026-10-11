/**
 * @file    mcp2515.h
 * @brief   Microchip MCP2515 stand-alone CAN controller over SPI.
 *
 * SPI mode 0, max 10 MHz. The driver uses the dedicated SPI instructions
 * (RESET, READ, WRITE, BIT MODIFY, LOAD TX BUFFER, RTS, READ RX BUFFER) and
 * only one transmit buffer (TXB0), which is enough for periodic telemetry.
 * Both receive buffers are open (masks / filters off, any ID accepted).
 *
 * Bit timing is computed from the crystal of the module (8 or 16 MHz on the
 * usual MCP2515 + TJA1050 boards) and the requested bit rate:
 *   bit time = N time quanta, TQ = 2 * (BRP + 1) / Fosc,
 *   N = 1 (sync) + PropSeg + PS1 + PS2, sample point about 75 %.
 */
#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "bus.h"

#define CAN_MAX_DLC        8U
#define CAN_ID_EXT_FLAG    0x80000000UL   /* set in id for a 29-bit identifier */

typedef struct {
    uint32_t id;                  /* 11-bit, or 29-bit | CAN_ID_EXT_FLAG */
    uint8_t  dlc;                 /* 0..8 */
    uint8_t  data[CAN_MAX_DLC];
} can_frame_t;

typedef enum {
    MCP2515_MODE_NORMAL   = 0x00, /* REQOP bits of CANCTRL */
    MCP2515_MODE_LOOPBACK = 0x40, /* frames go straight back to RX, nothing on the bus */
    MCP2515_MODE_LISTEN   = 0x60,
    MCP2515_MODE_CONFIG   = 0x80,
} mcp2515_mode_t;

typedef struct {
    uint8_t cnf1, cnf2, cnf3;     /* bit timing registers */
} mcp2515_timing_t;

typedef struct {
    spi_bus_t   bus;
    delay_ms_fn delay_ms;
    uint32_t    osc_hz;
    uint32_t    bitrate;
} mcp2515_t;

typedef struct {
    uint8_t tec, rec;             /* transmit / receive error counters */
    uint8_t eflg;                 /* error flags register */
} mcp2515_errors_t;

/** Compute CNF1..3. ERR_PARAM if the bit rate cannot be reached exactly. */
int mcp2515_calc_timing(uint32_t osc_hz, uint32_t bitrate, mcp2515_timing_t *out);

/** Reset, check the chip answers, set bit timing, open RX buffers, enter mode.
 *  ERR_NOT_FOUND if no MCP2515 responds (e.g. MISO floating at 0xFF). */
int mcp2515_init(mcp2515_t *dev, const spi_bus_t *bus, delay_ms_fn delay_ms,
                 uint32_t osc_hz, uint32_t bitrate, mcp2515_mode_t mode);

/** Queue one frame in TXB0. ERR_BUSY if the previous frame is still pending. */
int mcp2515_send(mcp2515_t *dev, const can_frame_t *f);

/** Read one received frame (RXB0 first). ERR_EMPTY if nothing arrived. */
int mcp2515_receive(mcp2515_t *dev, can_frame_t *f);

int mcp2515_read_errors(mcp2515_t *dev, mcp2515_errors_t *e);

#endif /* MCP2515_H */
