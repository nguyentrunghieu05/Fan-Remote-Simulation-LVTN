#include "can_link.h"
#include "mcp2515.h"
#include "sys_data.h"
#include "supervisor.h"
#include "bsp.h"
#include "board.h"
#include "err.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CAN_PERIOD_MS     1000U
#define CAN_RETRY_MS      10000U
#define TX_WAIT_MS        20U       /* 250 kbit/s: one 8-byte frame takes ~0.5 ms */
#define COB_STATUS1       0x180U
#define COB_STATUS2       0x280U

static mcp2515_t        s_can;
static spi_bus_t        s_bus;
static can_link_stats_t s_stats;

static bool try_init(void)
{
    int rc = mcp2515_init(&s_can, &s_bus, bsp_delay_ms, CAN_OSC_HZ, CAN_BITRATE, MCP2515_MODE_NORMAL);
    s_stats.present = (rc == ERR_OK);
    return s_stats.present;
}

void can_link_init(void)
{
    bsp_spi_can_bus(&s_bus);
    if (try_init()) {
        log_printf("CAN", "MCP2515 found, %lu kbit/s, %lu MHz crystal",
                   (unsigned long)(CAN_BITRATE / 1000U), (unsigned long)(CAN_OSC_HZ / 1000000U));
    } else {
        log_printf("CAN", "no MCP2515 found");
    }
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)v);
    put16(p + 2, (uint16_t)(v >> 16));
}

static void build_frames(const sys_data_t *d, can_frame_t *f1, can_frame_t *f2)
{
    uint32_t node = d->cfg.node_id & 0x7FU;
    memset(f1, 0, sizeof *f1);
    memset(f2, 0, sizeof *f2);

    f1->id  = COB_STATUS1 + node;
    f1->dlc = 8U;
    int16_t t = d->m.temp_valid ? (int16_t)(d->m.temp_c * 100.0f) : (int16_t)0x8000;
    put16(&f1->data[0], (uint16_t)t);
    put16(&f1->data[2], d->m.temp_valid ? (uint16_t)(d->m.hum_pct * 100.0f) : 0U);
    put16(&f1->data[4], d->m.rpm);
    put16(&f1->data[6], (uint16_t)(d->m.duty * 10.0f + 0.5f));

    f2->id  = COB_STATUS2 + node;
    f2->dlc = 8U;
    f2->data[0] = (uint8_t)d->m.state;
    f2->data[1] = (uint8_t)d->m.faults;
    f2->data[2] = (uint8_t)((d->m.temp_valid ? 1U : 0U) | (d->m.flash_ok ? 2U : 0U) | (d->m.oled_ok ? 4U : 0U));
    put32(&f2->data[4], bsp_millis() / 1000U);
}

/* The MCP2515 has one TX buffer in use: wait a little for the previous frame. */
static void send(const can_frame_t *f)
{
    for (uint32_t t = 0U; ; t++) {
        int rc = mcp2515_send(&s_can, f);
        if (rc == ERR_OK) {
            s_stats.sent++;
            return;
        }
        if (rc != ERR_BUSY || t >= TX_WAIT_MS) {
            s_stats.busy++;                      /* no ACK on the bus, or SPI error */
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void can_link_task(void *arg)
{
    (void)arg;
    int sv = sv_register("can", 3000U);
    TickType_t last = xTaskGetTickCount();
    uint32_t since_retry = 0U;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CAN_PERIOD_MS));
        sv_kick(sv);

        if (!s_stats.present) {
            since_retry += CAN_PERIOD_MS;
            if (since_retry >= CAN_RETRY_MS) {
                since_retry = 0U;
                if (try_init()) {
                    log_printf("CAN", "MCP2515 found");
                }
            }
            continue;
        }

        sys_data_t d;
        can_frame_t f1, f2, rx;
        sys_snapshot(&d);
        build_frames(&d, &f1, &f2);
        send(&f1);
        send(&f2);

        while (mcp2515_receive(&s_can, &rx) == ERR_OK) {
            s_stats.received++;                  /* nothing is listened to yet */
        }
        mcp2515_errors_t e;
        if (mcp2515_read_errors(&s_can, &e) == ERR_OK) {
            s_stats.tec = e.tec;
            s_stats.rec = e.rec;
        }
    }
}

void can_link_stats(can_link_stats_t *out)
{
    taskENTER_CRITICAL();
    *out = s_stats;
    taskEXIT_CRITICAL();
}
