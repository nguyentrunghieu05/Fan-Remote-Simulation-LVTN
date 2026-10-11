/**
 * @file    main.c
 * @brief   Boot sequence and task table of the fan controller.
 *
 *  Task        Prio  Period / trigger         Job
 *  ----------  ----  -----------------------  ------------------------------
 *  control      5    100 ms                   state machine, PID, outputs
 *  modbus       4    RS485 frame end (IRQ)    Modbus RTU slave
 *  sensors      3    10 ms                    button, knob, tach, SHT31
 *  link         2    LoRa RX (IRQ) / 100 ms   telemetry upload, commands
 *  storage      2    1 s                      data log on SPI flash
 *  can          2    1 s                      fan data on CAN (MCP2515)
 *  display      1    500 ms                   OLED status screen
 *  console      1    UART RX (IRQ)            command line
 *  supervisor   1    250 ms                   per-task watchdog -> IWDG
 *  (init        6    once at boot, then deleted)
 */
#include "stm32g0xx_hal.h"
#include "bsp.h"
#include "err.h"
#include "log.h"
#include "sys_data.h"
#include "storage.h"
#include "control.h"
#include "sensors.h"
#include "display.h"
#include "modbus_server.h"
#include "radio_link.h"
#include "can_link.h"
#include "console.h"
#include "supervisor.h"
#include "FreeRTOS.h"
#include "task.h"

#define FW_VERSION  "1.0.0"

static void start(TaskFunction_t fn, const char *name, uint16_t stack_words, UBaseType_t prio)
{
    if (xTaskCreate(fn, name, stack_words, NULL, prio, NULL) != pdPASS) {
        log_printf("SYS", "cannot create task %s", name);
        for (;;) { }
    }
}

/*
 * Why an init task: in FreeRTOS, any kernel call made before the scheduler
 * runs (creating a mutex, a queue, a task) leaves interrupts masked until
 * vTaskStartScheduler(). HAL_Delay() and the UART RX interrupts would then
 * stop working. So main() only sets up the clock and the outputs, and
 * everything that needs interrupts or delays is done here, with the
 * scheduler already running.
 */
static void init_task(void *arg)
{
    (void)arg;

    /* 1. console first, so every following step can report */
    console_init();
    log_printf("SYS", "fan controller v%s, %lu MHz, reset: %s",
               FW_VERSION, (unsigned long)(SystemCoreClock / 1000000U), bsp_reset_cause());

    /* 2. settings: from SPI flash if present, defaults otherwise */
    fan_config_t cfg;
    int rc = storage_init();
    if (rc == ERR_OK && storage_load_config(&cfg) == ERR_OK) {
        if (fan_config_sanitize(&cfg)) {
            log_printf("CFG", "stored settings out of range, corrected");
        }
        log_printf("CFG", "settings loaded from flash");
    } else {
        fan_config_defaults(&cfg);
        log_printf("CFG", "using default settings");
    }
    sys_data_init(&cfg);
    sys_lock();
    sys_data()->m.flash_ok = storage_ok();
    sys_unlock();

    /* 3. I2C bus shared by the sensor and the display */
    static i2c_bus_t i2c;
    bsp_i2c_init(&i2c);
    sensors_init(&i2c);
    display_init(&i2c);

    /* 4. communication and control */
    modbus_server_init();
    radio_link_init();
    can_link_init();                /* after storage_init(): shares its SPI bus */
    control_init();

    /* 5. application tasks (stack in 32-bit words) */
    start(control_task,       "control",  384, 5);
    start(modbus_server_task, "modbus",   384, 4);
    start(sensors_task,       "sensors",  384, 3);
    start(radio_link_task,    "link",     512, 2);
    start(storage_task,       "storage",  384, 2);
    start(can_link_task,      "can",      320, 2);
    start(display_task,       "display",  384, 1);
    start(console_task,       "console",  512, 1);
    start(sv_task,            "superv",   256, 1);

    log_printf("SYS", "running, heap free %u bytes", (unsigned)xPortGetFreeHeapSize());
    vTaskDelete(NULL);
}

int main(void)
{
    HAL_Init();
    bsp_clock_init();
    bsp_gpio_init();
    bsp_fan_out_init();             /* outputs at 0 % as early as possible */
    bsp_tach_init();
    bsp_adc_init();
    bsp_rtc_init();

    /* highest priority: runs alone until the other tasks exist */
    start(init_task, "init", 768, configMAX_PRIORITIES - 1);
    vTaskStartScheduler();
    for (;;) { }                    /* only reached if the heap is too small */
}
