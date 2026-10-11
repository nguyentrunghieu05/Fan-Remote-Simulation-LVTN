#include "radio_link.h"
#include "telemetry.h"
#include "storage.h"
#include "sys_data.h"
#include "supervisor.h"
#include "board.h"
#include "bsp.h"
#include "err.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define ACK_TIMEOUT_MS   2000U
#define SEND_GAP_MS      500U

static TaskHandle_t s_task;
static uint32_t     s_sent, s_acked, s_cmds;

static void on_rx(bsp_uart_t port)
{
    (void)port;
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void radio_link_init(void)
{
    bsp_lora_set_mode(0U, 0U);                 /* E220: M0=M1=0 transparent mode */
    bsp_uart_init(BSP_UART_LORA, LORA_BAUD, on_rx);
}

static void apply_cmd(const tlm_msg_t *m)
{
    s_cmds++;
    if (strcmp(m->name, "mode") == 0 && m->value >= 0 && m->value <= 2) {
        static const app_event_t ev[3] = { APP_EV_MODE_OFF, APP_EV_MODE_MANUAL, APP_EV_MODE_AUTO };
        (void)app_post_event(ev[m->value]);
    } else if (strcmp(m->name, "duty") == 0 && m->value >= 0 && m->value <= 1000) {
        sys_lock(); sys_data()->cfg.manual_duty_c10 = (uint16_t)m->value; sys_data()->cfg.use_pot = 0U; sys_unlock();
    } else if (strcmp(m->name, "setpoint") == 0 && m->value >= 0 && m->value <= 8000) {
        sys_lock(); sys_data()->cfg.setpoint_c100 = (int16_t)m->value; sys_unlock();
    } else if (strcmp(m->name, "time") == 0 && m->value > 0) {
        bsp_rtc_set_unix((uint32_t)m->value);   /* gateway has NTP: sync our clock */
    } else {
        log_printf("LINK", "unknown cmd %s=%ld", m->name, (long)m->value);
        return;
    }
    log_printf("LINK", "cmd %s=%ld", m->name, (long)m->value);
}

void radio_link_task(void *arg)
{
    (void)arg;
    s_task = xTaskGetCurrentTaskHandle();
    int sv = sv_register("link", 5000U);

    char     rx_line[TLM_LINE_MAX];
    size_t   rx_len = 0U;
    bool     waiting = false;
    uint32_t wait_seq = 0U, wait_slot = 0U, sent_at = 0U, next_at = 0U;

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        sv_kick(sv);
        uint32_t now = bsp_millis();

        /* 1. assemble received lines */
        char c;
        while (bsp_uart_read(BSP_UART_LORA, &c, 1U) == 1U) {
            if (c == '\n' || c == '\r') {
                if (rx_len > 0U) {
                    rx_line[rx_len] = '\0';
                    tlm_msg_t m;
                    sys_data_t d;
                    sys_snapshot(&d);
                    if (tlm_parse(rx_line, &m) == ERR_OK && m.node == d.cfg.node_id) {
                        if (m.kind == TLM_ACK && waiting && m.seq == wait_seq) {
                            (void)storage_log_mark_sent(wait_slot);
                            s_acked++;
                            waiting = false;
                            next_at = now + SEND_GAP_MS;      /* link is up: keep going */
                        } else if (m.kind == TLM_CMD) {
                            apply_cmd(&m);
                        }
                    }
                }
                rx_len = 0U;
            } else if (rx_len < sizeof rx_line - 1U) {
                rx_line[rx_len++] = c;
            }
        }

        /* 2. no ACK in time: the gateway is out of reach, retry later */
        if (waiting && now - sent_at > ACK_TIMEOUT_MS) {
            waiting = false;
            sys_data_t d;
            sys_snapshot(&d);
            next_at = now + d.cfg.tlm_period_s * 1000U;
        }

        /* 3. send the next record */
        if (!waiting && (int32_t)(now - next_at) >= 0) {
            sys_data_t d;
            sys_snapshot(&d);
            datalog_sample_t s;
            char line[TLM_LINE_MAX];
            int rc = storage_log_next_unsent(&wait_slot, &wait_seq, &s);
            if (rc == ERR_OK) {
                waiting = true;
            } else if (storage_ok()) {
                next_at = now + 1000U;      /* all uploaded: look again in 1 s */
                continue;
            } else {
                /* no flash: send a live sample every period, no ACK needed */
                s.timestamp = bsp_rtc_get_unix();
                s.temp_c100 = d.m.temp_valid ? (int16_t)(d.m.temp_c * 100.0f) : INT16_MIN;
                s.hum_c100  = (uint16_t)(d.m.hum_pct * 100.0f);
                s.rpm       = d.m.rpm;
                s.duty_c10  = (uint16_t)(d.m.duty * 10.0f + 0.5f);
                s.state     = (uint8_t)d.m.state;
                s.faults    = (uint8_t)d.m.faults;
                wait_seq    = 0U;
                next_at     = now + d.cfg.tlm_period_s * 1000U;
            }
            size_t n = tlm_encode_sample(line, sizeof line, d.cfg.node_id, wait_seq, &s);
            if (n > 0U) {
                (void)bsp_uart_write(BSP_UART_LORA, line, n);
                s_sent++;
            }
            sent_at = now;
        }
    }
}

void radio_link_stats(uint32_t *sent, uint32_t *acked, uint32_t *rx_cmds)
{
    *sent    = s_sent;
    *acked   = s_acked;
    *rx_cmds = s_cmds;
}
