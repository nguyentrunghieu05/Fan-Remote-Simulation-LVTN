/**
 * @file    modbus_slave.h
 * @brief   Modbus RTU slave protocol engine (transport independent).
 *
 * The UART driver detects the end of a frame (3.5 character silence, done
 * in hardware with the USART receiver-timeout) and hands the whole frame to
 * mb_slave_process(). This module checks address + CRC, executes the
 * function and builds the response. Register storage is not here: the
 * application supplies read/write callbacks, so the protocol code knows
 * nothing about fans.
 *
 * Supported functions: 0x03 Read Holding, 0x04 Read Input,
 *                      0x06 Write Single, 0x10 Write Multiple.
 */
#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include <stdint.h>
#include <stddef.h>

#define MB_FRAME_MAX        256U
#define MB_ADDR_BROADCAST   0U
#define MB_MAX_READ_REGS    125U
#define MB_MAX_WRITE_REGS   123U

/* Exception codes (0 = no exception). */
typedef enum {
    MB_EX_NONE              = 0x00,
    MB_EX_ILLEGAL_FUNCTION  = 0x01,
    MB_EX_ILLEGAL_ADDRESS   = 0x02,
    MB_EX_ILLEGAL_VALUE     = 0x03,
    MB_EX_DEVICE_FAILURE    = 0x04,
} mb_exception_t;

typedef enum {
    MB_TABLE_INPUT   = 0,   /* read-only measurements (FC 04) */
    MB_TABLE_HOLDING = 1,   /* read/write settings    (FC 03/06/10) */
} mb_table_t;

typedef struct {
    mb_exception_t (*read_reg)(void *ctx, mb_table_t table, uint16_t addr, uint16_t *value);
    mb_exception_t (*write_reg)(void *ctx, uint16_t addr, uint16_t value);
    void *ctx;
} mb_callbacks_t;

typedef struct {
    uint8_t               address;     /* 1..247 */
    const mb_callbacks_t *cb;
    /* statistics, useful for the thesis test chapter */
    uint32_t rx_frames;
    uint32_t crc_errors;
    uint32_t exceptions;
} mb_slave_t;

void mb_slave_init(mb_slave_t *s, uint8_t address, const mb_callbacks_t *cb);

/**
 * Process one received frame.
 * @param resp      buffer of at least MB_FRAME_MAX bytes
 * @return length of the response to send, 0 if nothing must be sent
 *         (wrong address, bad CRC, broadcast).
 */
size_t mb_slave_process(mb_slave_t *s, const uint8_t *req, size_t len, uint8_t *resp);

#endif /* MODBUS_SLAVE_H */
