/**
 * @file    can_link.h
 * @brief   Periodic fan data on a CAN bus through the MCP2515 (SPI).
 *
 * Every second two 8-byte frames are sent, little endian, with IDs in the
 * style of CANopen TPDOs so a PLC or a USB-CAN adapter can decode them:
 *
 *   0x180 + node  temp (0.01 C, int16, 0x8000 = no sensor), humidity
 *                 (0.01 %, uint16), fan speed (rpm, uint16), output (0.1 %, uint16)
 *   0x280 + node  state (uint8), faults (uint8), flags (uint8: bit0 temp valid,
 *                 bit1 flash ok, bit2 display ok), 0, uptime (s, uint32)
 *
 * Without an MCP2515 the task keeps running and looks for it every 10 s.
 */
#ifndef CAN_LINK_H
#define CAN_LINK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool     present;
    uint32_t sent;        /* frames accepted by the controller   */
    uint32_t busy;        /* frames dropped: TX buffer still full */
    uint32_t received;    /* frames read from the bus (ignored)   */
    uint8_t  tec, rec;    /* MCP2515 error counters               */
} can_link_stats_t;

void can_link_init(void);
void can_link_task(void *arg);
void can_link_stats(can_link_stats_t *out);

#endif /* CAN_LINK_H */
