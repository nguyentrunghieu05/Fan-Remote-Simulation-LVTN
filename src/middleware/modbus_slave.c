#include "modbus_slave.h"
#include "crc.h"

static uint16_t get_u16(const uint8_t *p)          { return (uint16_t)((p[0] << 8) | p[1]); }
static void     put_u16(uint8_t *p, uint16_t v)    { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

void mb_slave_init(mb_slave_t *s, uint8_t address, const mb_callbacks_t *cb)
{
    s->address    = address;
    s->cb         = cb;
    s->rx_frames  = 0U;
    s->crc_errors = 0U;
    s->exceptions = 0U;
}

static size_t finish(uint8_t *resp, size_t len)
{
    uint16_t crc = crc16_modbus(resp, len);
    resp[len]     = (uint8_t)(crc & 0xFFU);   /* CRC is sent low byte first */
    resp[len + 1] = (uint8_t)(crc >> 8);
    return len + 2U;
}

static size_t exception(mb_slave_t *s, uint8_t *resp, uint8_t fc, mb_exception_t ex)
{
    s->exceptions++;
    resp[1] = (uint8_t)(fc | 0x80U);
    resp[2] = (uint8_t)ex;
    return finish(resp, 3U);
}

static mb_exception_t do_read(mb_slave_t *s, mb_table_t t, const uint8_t *pdu, uint8_t *resp, size_t *out)
{
    uint16_t start = get_u16(&pdu[1]);
    uint16_t qty   = get_u16(&pdu[3]);
    if (qty == 0U || qty > MB_MAX_READ_REGS) {
        return MB_EX_ILLEGAL_VALUE;
    }
    if ((uint32_t)start + qty > 0x10000UL) {
        return MB_EX_ILLEGAL_ADDRESS;
    }
    resp[2] = (uint8_t)(qty * 2U);
    for (uint16_t i = 0U; i < qty; i++) {
        uint16_t v = 0U;
        mb_exception_t ex = s->cb->read_reg(s->cb->ctx, t, (uint16_t)(start + i), &v);
        if (ex != MB_EX_NONE) {
            return ex;
        }
        put_u16(&resp[3U + 2U * i], v);
    }
    *out = 3U + 2U * qty;
    return MB_EX_NONE;
}

static mb_exception_t do_write_single(mb_slave_t *s, const uint8_t *pdu, uint8_t *resp, size_t *out)
{
    mb_exception_t ex = s->cb->write_reg(s->cb->ctx, get_u16(&pdu[1]), get_u16(&pdu[3]));
    if (ex != MB_EX_NONE) {
        return ex;
    }
    for (int i = 1; i < 5; i++) {
        resp[i + 1] = pdu[i];                  /* echo address + value */
    }
    *out = 6U;
    return MB_EX_NONE;
}

static mb_exception_t do_write_multiple(mb_slave_t *s, const uint8_t *pdu, size_t pdu_len,
                                        uint8_t *resp, size_t *out)
{
    uint16_t start = get_u16(&pdu[1]);
    uint16_t qty   = get_u16(&pdu[3]);
    uint8_t  bytes = pdu[5];
    if (qty == 0U || qty > MB_MAX_WRITE_REGS || bytes != qty * 2U || pdu_len < 6U + bytes) {
        return MB_EX_ILLEGAL_VALUE;
    }
    if ((uint32_t)start + qty > 0x10000UL) {
        return MB_EX_ILLEGAL_ADDRESS;
    }
    for (uint16_t i = 0U; i < qty; i++) {
        mb_exception_t ex = s->cb->write_reg(s->cb->ctx, (uint16_t)(start + i),
                                             get_u16(&pdu[6U + 2U * i]));
        if (ex != MB_EX_NONE) {
            return ex;
        }
    }
    put_u16(&resp[2], start);
    put_u16(&resp[4], qty);
    *out = 6U;
    return MB_EX_NONE;
}

size_t mb_slave_process(mb_slave_t *s, const uint8_t *req, size_t len, uint8_t *resp)
{
    if (len < 4U || len > MB_FRAME_MAX) {
        return 0U;
    }
    uint8_t addr = req[0];
    if (addr != s->address && addr != MB_ADDR_BROADCAST) {
        return 0U;                              /* not for us: stay silent */
    }
    uint16_t rx_crc = (uint16_t)(req[len - 2U] | (req[len - 1U] << 8));
    if (crc16_modbus(req, len - 2U) != rx_crc) {
        s->crc_errors++;
        return 0U;                              /* spec: ignore corrupted frames */
    }
    s->rx_frames++;

    const uint8_t *pdu     = &req[1];
    size_t         pdu_len = len - 3U;
    uint8_t        fc      = pdu[0];
    size_t         out     = 0U;
    mb_exception_t ex;

    resp[0] = s->address;
    resp[1] = fc;

    switch (fc) {
    case 0x03:
    case 0x04:
        ex = (pdu_len >= 5U)
           ? do_read(s, fc == 0x03 ? MB_TABLE_HOLDING : MB_TABLE_INPUT, pdu, resp, &out)
           : MB_EX_ILLEGAL_VALUE;
        break;
    case 0x06:
        ex = (pdu_len >= 5U) ? do_write_single(s, pdu, resp, &out) : MB_EX_ILLEGAL_VALUE;
        break;
    case 0x10:
        ex = (pdu_len >= 6U) ? do_write_multiple(s, pdu, pdu_len, resp, &out) : MB_EX_ILLEGAL_VALUE;
        break;
    default:
        ex = MB_EX_ILLEGAL_FUNCTION;
        break;
    }

    if (addr == MB_ADDR_BROADCAST) {
        return 0U;                              /* broadcasts are never answered */
    }
    if (ex != MB_EX_NONE) {
        return exception(s, resp, fc, ex);
    }
    return finish(resp, out);
}
