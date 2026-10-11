#include "telemetry.h"
#include "err.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint8_t tlm_checksum(const char *body, size_t len)
{
    uint8_t cs = 0U;
    for (size_t i = 0U; i < len; i++) {
        cs ^= (uint8_t)body[i];
    }
    return cs;
}

size_t tlm_encode_sample(char *out, size_t cap, uint16_t node, uint32_t seq, const datalog_sample_t *s)
{
    char body[TLM_LINE_MAX];
    int n = snprintf(body, sizeof body, "FAN,%u,%lu,%lu,%d,%u,%u,%u,%u,%u",
                     (unsigned)node, (unsigned long)seq, (unsigned long)s->timestamp,
                     (int)s->temp_c100, (unsigned)s->hum_c100, (unsigned)s->rpm,
                     (unsigned)s->duty_c10, (unsigned)s->state, (unsigned)s->faults);
    if (n <= 0 || (size_t)n >= sizeof body) {
        return 0U;
    }
    int m = snprintf(out, cap, "$%s*%02X\r\n", body, tlm_checksum(body, (size_t)n));
    return (m > 0 && (size_t)m < cap) ? (size_t)m : 0U;
}

int tlm_parse(const char *line, tlm_msg_t *msg)
{
    memset(msg, 0, sizeof *msg);
    const char *star = strchr(line, '*');
    if (line[0] != '$' || star == NULL || strlen(star) < 3U) {
        return ERR_PARAM;
    }
    size_t body_len = (size_t)(star - line - 1);
    char  *end = NULL;
    unsigned long cs = strtoul(star + 1, &end, 16);
    if (end != star + 3 || cs != tlm_checksum(line + 1, body_len)) {
        return ERR_CRC;
    }

    char body[TLM_LINE_MAX];
    if (body_len >= sizeof body) {
        return ERR_PARAM;
    }
    memcpy(body, line + 1, body_len);
    body[body_len] = '\0';

    char *fields[5] = { 0 };
    int   nf = 0;
    for (char *tok = strtok(body, ","); tok != NULL && nf < 5; tok = strtok(NULL, ",")) {
        fields[nf++] = tok;
    }
    if (nf >= 3 && strcmp(fields[0], "ACK") == 0) {
        msg->kind = TLM_ACK;
        msg->node = (uint16_t)strtoul(fields[1], NULL, 10);
        msg->seq  = strtoul(fields[2], NULL, 10);
        return ERR_OK;
    }
    if (nf >= 4 && strcmp(fields[0], "CMD") == 0) {
        msg->kind  = TLM_CMD;
        msg->node  = (uint16_t)strtoul(fields[1], NULL, 10);
        strncpy(msg->name, fields[2], sizeof msg->name - 1U);
        msg->value = (int32_t)strtol(fields[3], NULL, 10);
        return ERR_OK;
    }
    return ERR_PARAM;
}
