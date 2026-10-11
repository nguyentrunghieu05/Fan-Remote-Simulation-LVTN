/**
 * @file    radio_link.h
 * @brief   Store-and-forward telemetry over a LoRa UART module.
 *
 * Every record in the flash log is sent to the gateway, oldest unsent
 * first, and marked as sent only when the gateway answers $ACK with the
 * same sequence number. If the link is down the data simply waits in
 * flash; when it comes back, the backlog is uploaded at about 2 records/s.
 * The gateway can also send $CMD lines (mode, duty, setpoint, time).
 */
#ifndef RADIO_LINK_H
#define RADIO_LINK_H

#include <stdint.h>

void radio_link_init(void);
void radio_link_task(void *arg);
void radio_link_stats(uint32_t *sent, uint32_t *acked, uint32_t *rx_cmds);

#endif /* RADIO_LINK_H */
