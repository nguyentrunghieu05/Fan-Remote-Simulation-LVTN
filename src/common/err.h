/**
 * @file    err.h
 * @brief   Common return codes shared by every layer.
 *
 * Convention: 0 = success, negative = error. Functions that return a count
 * return it as a non-negative int.
 */
#ifndef ERR_H
#define ERR_H

enum {
    ERR_OK        = 0,
    ERR_PARAM     = -1,   /* invalid argument                         */
    ERR_TIMEOUT   = -2,   /* peripheral or device did not answer      */
    ERR_IO        = -3,   /* bus error, NACK, framing error           */
    ERR_CRC       = -4,   /* data received but checksum is wrong      */
    ERR_NOT_FOUND = -5,   /* device / record not present              */
    ERR_FULL      = -6,   /* buffer or storage full                   */
    ERR_EMPTY     = -7,   /* nothing to read                          */
    ERR_BUSY      = -8,   /* resource locked by someone else          */
    ERR_STATE     = -9,   /* operation not allowed in current state   */
};

#endif /* ERR_H */
