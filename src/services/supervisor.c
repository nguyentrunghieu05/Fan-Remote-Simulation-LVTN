#include "supervisor.h"
#include "bsp.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

#define IWDG_TIMEOUT_MS   2000U
#define SV_PERIOD_MS      250U

typedef struct {
    const char       *name;
    uint32_t          timeout_ms;
    volatile uint32_t last_ms;
} sv_client_t;

static sv_client_t s_cli[SV_MAX_CLIENTS];
static int         s_count;
static volatile bool s_maint;

void sv_maintenance(bool on)
{
    s_maint = on;
    if (!on) {
        for (int i = 0; i < s_count; i++) {
            s_cli[i].last_ms = bsp_millis();     /* fresh start for every task */
        }
    }
}

int sv_register(const char *name, uint32_t timeout_ms)
{
    taskENTER_CRITICAL();
    int id = (s_count < SV_MAX_CLIENTS) ? s_count++ : -1;
    if (id >= 0) {
        s_cli[id].name       = name;
        s_cli[id].timeout_ms = timeout_ms;
        s_cli[id].last_ms    = bsp_millis();
    }
    taskEXIT_CRITICAL();
    return id;
}

void sv_kick(int id)
{
    if (id >= 0 && id < s_count) {
        s_cli[id].last_ms = bsp_millis();
    }
}

void sv_task(void *arg)
{
    (void)arg;
    bsp_iwdg_init(IWDG_TIMEOUT_MS);
    bool reported = false;
    for (;;) {
        uint32_t now = bsp_millis();
        int late = -1;
        for (int i = 0; !s_maint && i < s_count; i++) {
            if (now - s_cli[i].last_ms > s_cli[i].timeout_ms) {
                late = i;
                break;
            }
        }
        if (late < 0) {
            bsp_iwdg_refresh();
        } else if (!reported) {
            /* Stop feeding: the IWDG will reset the MCU within 2 s. */
            log_printf("WDG", "task '%s' missed its %lu ms deadline -> reset",
                       s_cli[late].name, (unsigned long)s_cli[late].timeout_ms);
            reported = true;
        }
        vTaskDelay(pdMS_TO_TICKS(SV_PERIOD_MS));
    }
}
