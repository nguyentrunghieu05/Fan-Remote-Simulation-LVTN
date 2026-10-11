/**
 * @file    telemetry.h
 * @brief   Text telemetry frames sent over the radio link (LoRa) or USB.
 *
 * NMEA-like, human readable and easy to parse on a gateway (ESP32,
 * Raspberry Pi, Node-RED):
 *
 *   $FAN,<node>,<seq>,<unix>,<t_c100>,<rh_c100>,<rpm>,<duty_c10>,<state>,<faults>*CS\r\n
 *   $ACK,<node>,<seq>*CS\r\n                       (gateway -> controller)
 *   $CMD,<node>,<name>,<value>*CS\r\n               (gateway -> controller)
 *
 * CS = XOR of all characters between '$' and '*', two hex digits.
 * The ACK lets the controller mark a stored record as uploaded.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stddef.h>
#include "datalog.h"

#define TLM_LINE_MAX  128U

typedef enum { TLM_NONE = 0, TLM_ACK, TLM_CMD } tlm_kind_t;

typedef struct {
    tlm_kind_t kind;
    uint16_t   node;
    uint32_t   seq;          /* ACK */
    char       name[12];     /* CMD: "mode", "duty", "setpoint" */
    int32_t    value;        /* CMD */
} tlm_msg_t;

/** @return length written (without NUL), 0 if buffer too small. */
size_t tlm_encode_sample(char *out, size_t cap, uint16_t node, uint32_t seq, const datalog_sample_t *s);
/** Parse one received line (without CR/LF). @return 0 ok, <0 error (format or checksum). */
int    tlm_parse(const char *line, tlm_msg_t *msg);
uint8_t tlm_checksum(const char *body, size_t len);

#endif /* TELEMETRY_H */
