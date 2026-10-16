#include "storage.h"
#include "w25qxx.h"
#include "config_store.h"
#include "bsp.h"
#include "board.h"
#include "err.h"
#include "log.h"
#include "sys_data.h"
#include "supervisor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define FW_CONFIRM_S  30U

static w25qxx_t          s_chip;
static flash_dev_t       s_flash;
static datalog_t         s_log;
static config_store_t    s_cfg;
static config_store_t    s_fwst;
static fw_writer_t       s_fw;
static SemaphoreHandle_t s_lock;
static bool              s_ok;

#define LOCK()    do { if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) xSemaphoreTake(s_lock, portMAX_DELAY); } while (0)
#define UNLOCK()  do { if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) xSemaphoreGive(s_lock); } while (0)

int storage_init(void)
{
    spi_bus_t bus;
    s_lock = xSemaphoreCreateMutex();
    bsp_spi_init(&bus);

    int rc = w25qxx_init(&s_chip, &bus, bsp_delay_ms);
    if (rc != ERR_OK) {
        log_printf("FLASH", "no SPI flash found (rc=%d), running without storage", rc);
        return rc;
    }
    w25qxx_as_flash(&s_chip, &s_flash);
    config_store_init(&s_cfg, &s_flash, STORAGE_CFG_A_ADDR, STORAGE_CFG_B_ADDR);
    config_store_init(&s_fwst, &s_flash, FW_STATE_A_ADDR, FW_STATE_B_ADDR);

    uint32_t sectors = (s_flash.size - STORAGE_LOG_ADDR) / s_flash.sector_size;
    if (sectors > STORAGE_LOG_SECTORS_MAX) {
        sectors = STORAGE_LOG_SECTORS_MAX;
    }
    rc = datalog_init(&s_log, &s_flash, STORAGE_LOG_ADDR, sectors);
    if (rc != ERR_OK) {
        log_printf("FLASH", "log init failed rc=%d", rc);
        return rc;
    }
    s_ok = true;
    log_printf("FLASH", "JEDEC %06lX, %lu KB, log %lu/%lu records",
               (unsigned long)s_chip.jedec_id, (unsigned long)(s_flash.size / 1024U),
               (unsigned long)datalog_count(&s_log), (unsigned long)s_log.total_slots);
    return ERR_OK;
}

bool     storage_ok(void)         { return s_ok; }
uint32_t storage_jedec_id(void)   { return s_chip.jedec_id; }
uint32_t storage_size(void)       { return s_ok ? s_flash.size : 0U; }
uint32_t storage_log_capacity(void) { return s_ok ? s_log.total_slots : 0U; }

int storage_load_config(fan_config_t *cfg)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = config_store_load(&s_cfg, cfg, sizeof *cfg);
    UNLOCK();
    return rc;
}

int storage_save_config(const fan_config_t *cfg)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = config_store_save(&s_cfg, cfg, sizeof *cfg);
    UNLOCK();
    return rc;
}

int storage_log_append(const datalog_sample_t *s)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = datalog_append(&s_log, s);
    UNLOCK();
    return rc;
}

uint32_t storage_log_count(void)
{
    if (!s_ok) { return 0U; }
    LOCK();
    uint32_t n = datalog_count(&s_log);
    UNLOCK();
    return n;
}

int storage_log_read(uint32_t index, datalog_sample_t *s, uint32_t *seq)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = datalog_read(&s_log, index, s, seq);
    UNLOCK();
    return rc;
}

int storage_log_next_unsent(uint32_t *slot, uint32_t *seq, datalog_sample_t *s)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = datalog_next_unsent(&s_log, slot, seq, s);
    UNLOCK();
    return rc;
}

int storage_log_mark_sent(uint32_t slot)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = datalog_mark_sent(&s_log, slot);
    UNLOCK();
    return rc;
}

int storage_log_format(void)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = datalog_format(&s_log);
    UNLOCK();
    return rc;
}

/* ---------------------------------------------------------- firmware update */
int storage_fw_begin(void)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = fw_writer_begin(&s_fw, &s_flash, FW_SLOT_NEW, APP_MAX_SIZE);
    UNLOCK();
    return rc;
}

int storage_fw_put(const void *data, size_t len)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = fw_writer_put(&s_fw, data, len);
    UNLOCK();
    return rc;
}

int storage_fw_finish(uint32_t version, uint32_t crc)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = fw_writer_finish(&s_fw, version, crc);
    UNLOCK();
    return rc;
}

int storage_fw_slot(uint32_t slot, fw_header_t *h)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = fw_slot_verify(&s_flash, slot, APP_MAX_SIZE, h);
    UNLOCK();
    return rc;
}

static int fw_state_load(fw_boot_t *st)
{
    int rc = config_store_load(&s_fwst, st, sizeof *st);
    if (rc == ERR_NOT_FOUND) {
        memset(st, 0, sizeof *st);                     /* never updated: IDLE */
        rc = ERR_OK;
    }
    return rc;
}

int storage_fw_state(fw_boot_t *st)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    int rc = fw_state_load(st);
    UNLOCK();
    return rc;
}

int storage_fw_request_install(void)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    fw_boot_t st;
    int rc = fw_slot_verify(&s_flash, FW_SLOT_NEW, APP_MAX_SIZE, NULL);
    if (rc == ERR_OK) {
        rc = fw_state_load(&st);
    }
    if (rc == ERR_OK) {
        st.state = FW_STATE_PENDING;
        st.tries = 0U;
        rc = config_store_save(&s_fwst, &st, sizeof st);
    }
    UNLOCK();
    return rc;
}

int storage_fw_confirm(void)
{
    if (!s_ok) { return ERR_STATE; }
    LOCK();
    fw_boot_t st;
    int rc = fw_state_load(&st);
    if (rc == ERR_OK) {
        rc = fw_boot_confirm(&st) ? config_store_save(&s_fwst, &st, sizeof st) : ERR_STATE;
    }
    UNLOCK();
    return rc;
}

void storage_task(void *arg)
{
    (void)arg;
    int sv = sv_register("storage", 70000U);   /* a 64 KB erase can take a while */
    TickType_t last = xTaskGetTickCount();
    uint32_t elapsed_s = 0U;
    uint32_t uptime_s  = 0U;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
        sv_kick(sv);

        /* a new firmware that ran this long without a reset is kept */
        if (++uptime_s == FW_CONFIRM_S && s_ok && storage_fw_confirm() == ERR_OK) {
            log_printf("FW", "new firmware confirmed after %u s", FW_CONFIRM_S);
        }

        sys_data_t d;
        sys_snapshot(&d);
        if (++elapsed_s < d.cfg.log_period_s || !s_ok) {
            continue;
        }
        elapsed_s = 0U;

        datalog_sample_t s = {
            .timestamp = bsp_rtc_get_unix(),
            .temp_c100 = d.m.temp_valid ? (int16_t)(d.m.temp_c * 100.0f) : INT16_MIN,
            .hum_c100  = d.m.temp_valid ? (uint16_t)(d.m.hum_pct * 100.0f) : 0U,
            .rpm       = d.m.rpm,
            .duty_c10  = (uint16_t)(d.m.duty * 10.0f + 0.5f),
            .state     = (uint8_t)d.m.state,
            .faults    = (uint8_t)d.m.faults,
        };
        int rc = storage_log_append(&s);
        if (rc != ERR_OK) {
            log_printf("FLASH", "log append failed rc=%d", rc);
        }
    }
}
