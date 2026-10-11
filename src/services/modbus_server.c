#include "modbus_server.h"
#include "modbus_slave.h"
#include "sys_data.h"
#include "storage.h"
#include "supervisor.h"
#include "board.h"
#include "bsp.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

static mb_slave_t   s_mb;
static TaskHandle_t s_task;

static void on_frame_end(bsp_uart_t port)
{
    (void)port;
    BaseType_t woken = pdFALSE;
    if (s_task != NULL) {
        vTaskNotifyGiveFromISR(s_task, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

static mb_exception_t read_reg(void *ctx, mb_table_t table, uint16_t addr, uint16_t *v)
{
    (void)ctx;
    sys_data_t d;
    sys_snapshot(&d);
    const fan_config_t *c = &d.cfg;

    if (table == MB_TABLE_INPUT) {
        uint32_t up   = bsp_millis() / 1000U;
        uint32_t nlog = storage_log_count();
        switch (addr) {
        case 0:  *v = d.m.temp_valid ? (uint16_t)(int16_t)(d.m.temp_c * 100.0f) : 0x8000U; break;
        case 1:  *v = (uint16_t)(d.m.hum_pct * 100.0f); break;
        case 2:  *v = d.m.rpm; break;
        case 3:  *v = (uint16_t)(d.m.duty * 10.0f + 0.5f); break;
        case 4:  *v = (uint16_t)d.m.state; break;
        case 5:  *v = (uint16_t)d.m.faults; break;
        case 6:  *v = (uint16_t)(d.m.pot_percent * 10.0f); break;
        case 7:  *v = (uint16_t)(up >> 16); break;
        case 8:  *v = (uint16_t)up; break;
        case 9:  *v = (uint16_t)(nlog >> 16); break;
        case 10: *v = (uint16_t)nlog; break;
        case 11: *v = FW_VERSION_REG; break;
        default: return MB_EX_ILLEGAL_ADDRESS;
        }
        return MB_EX_NONE;
    }

    switch (addr) {
    case 0:  *v = (d.m.state == APP_STATE_FAULT) ? 3U : (uint16_t)d.m.state; break;
    case 1:  *v = c->manual_duty_c10; break;
    case 2:  *v = (uint16_t)c->setpoint_c100; break;
    case 3:  *v = (uint16_t)(c->kp * 100.0f + 0.5f); break;
    case 4:  *v = (uint16_t)(c->ki * 100.0f + 0.5f); break;
    case 5:  *v = (uint16_t)(c->kd * 100.0f + 0.5f); break;
    case 6:  *v = c->min_duty_c10; break;
    case 7:  *v = c->fault_duty_c10; break;
    case 8:  *v = (uint16_t)c->over_temp_c100; break;
    case 9:  *v = c->ramp_c10_per_s; break;
    case 10: *v = c->log_period_s; break;
    case 11: *v = c->tlm_period_s; break;
    case 12: *v = c->modbus_addr; break;
    case 13: *v = c->use_pot; break;
    case 14: *v = c->stall_rpm; break;
    case 20: *v = 0U; break;
    default: return MB_EX_ILLEGAL_ADDRESS;
    }
    return MB_EX_NONE;
}

static mb_exception_t write_reg(void *ctx, uint16_t addr, uint16_t v)
{
    (void)ctx;
    if (addr == 0U) {
        static const app_event_t ev[3] = { APP_EV_MODE_OFF, APP_EV_MODE_MANUAL, APP_EV_MODE_AUTO };
        if (v > 2U) {
            return MB_EX_ILLEGAL_VALUE;
        }
        return app_post_event(ev[v]) ? MB_EX_NONE : MB_EX_DEVICE_FAILURE;
    }
    if (addr == 20U) {
        switch (v) {
        case 1: {
            sys_data_t d;
            sys_snapshot(&d);
            return (storage_save_config(&d.cfg) == 0) ? MB_EX_NONE : MB_EX_DEVICE_FAILURE;
        }
        case 2: app_request_fault_reset(); return MB_EX_NONE;
        case 3: bsp_reset(); return MB_EX_NONE;
        default: return MB_EX_ILLEGAL_VALUE;
        }
    }

    mb_exception_t ex = MB_EX_NONE;
    sys_lock();
    fan_config_t *c = &sys_data()->cfg;
    switch (addr) {
    case 1:  if (v <= 1000U) { c->manual_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 2:  if (v <= 8000U) { c->setpoint_c100 = (int16_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 3:  c->kp = v / 100.0f; break;
    case 4:  c->ki = v / 100.0f; break;
    case 5:  c->kd = v / 100.0f; break;
    case 6:  if (v <= 500U)  { c->min_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 7:  if (v <= 1000U) { c->fault_duty_c10 = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 8:  if (v >= 2000U && v <= 12000U) { c->over_temp_c100 = (int16_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 9:  if (v >= 10U && v <= 1000U) { c->ramp_c10_per_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 10: if (v >= 1U && v <= 3600U) { c->log_period_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 11: if (v >= 5U && v <= 3600U) { c->tlm_period_s = v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 12: if (v >= 1U && v <= 247U)  { c->modbus_addr = (uint8_t)v; } else { ex = MB_EX_ILLEGAL_VALUE; } break;
    case 13: c->use_pot = (v != 0U); break;
    case 14: c->stall_rpm = v; break;
    default: ex = MB_EX_ILLEGAL_ADDRESS; break;
    }
    (void)fan_config_sanitize(c);
    sys_unlock();
    return ex;
}

static const mb_callbacks_t s_cb = { read_reg, write_reg, NULL };

void modbus_server_init(void)
{
    sys_data_t d;
    sys_snapshot(&d);
    mb_slave_init(&s_mb, d.cfg.modbus_addr, &s_cb);
    bsp_uart_init(BSP_UART_RS485, RS485_BAUD, on_frame_end);
    log_printf("MB", "RTU slave id %u, %lu baud 8N1", d.cfg.modbus_addr, (unsigned long)RS485_BAUD);
}

void modbus_server_task(void *arg)
{
    (void)arg;
    static uint8_t req[MB_FRAME_MAX], rsp[MB_FRAME_MAX];
    s_task = xTaskGetCurrentTaskHandle();
    int sv = sv_register("modbus", 3000U);

    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));   /* wakes on frame end or 1 s */
        sv_kick(sv);
        size_t n = bsp_uart_read(BSP_UART_RS485, req, sizeof req);
        if (n == 0U) {
            continue;
        }
        if (n == sizeof req) {
            bsp_uart_flush_rx(BSP_UART_RS485);   /* longer than any Modbus frame: garbage */
            continue;
        }
        size_t out = mb_slave_process(&s_mb, req, n, rsp);
        if (out > 0U) {
            (void)bsp_uart_write(BSP_UART_RS485, rsp, out);
        }
    }
}

void modbus_server_stats(uint32_t *frames, uint32_t *crc_err, uint32_t *exceptions)
{
    *frames     = s_mb.rx_frames;
    *crc_err    = s_mb.crc_errors;
    *exceptions = s_mb.exceptions;
}
