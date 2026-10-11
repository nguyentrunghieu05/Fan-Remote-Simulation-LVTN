/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS kernel configuration for STM32G0B1 (Cortex-M0+, 64 MHz).
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
extern uint32_t SystemCoreClock;

#define configUSE_PREEMPTION                    1
#define configENABLE_MPU                        0
#define configUSE_TIME_SLICING                  1
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                   ((size_t)(32 * 1024))
#define configMAX_TASK_NAME_LEN                 12
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_TASK_NOTIFICATIONS            1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configUSE_NEWLIB_REENTRANT              0

/* Hooks and diagnostics: used by the thesis test chapter (stack usage,
 * CPU load per task via the "tasks" CLI command). */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
#define configGENERATE_RUN_TIME_STATS           1
#define configRECORD_STACK_HIGH_ADDRESS         1
extern void     rtstats_timer_init(void);
extern uint32_t rtstats_timer_get(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() rtstats_timer_init()
#define portGET_RUN_TIME_COUNTER_VALUE()        rtstats_timer_get()

/* Software timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            256

/* API functions included */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetCurrentTaskHandle       1

/* Catch kernel misuse during development. */
#define configASSERT(x) do { if ((x) == 0) { __asm volatile ("cpsid i"); for (;;) {} } } while (0)

/* The Cortex-M0 port (FreeRTOS V11) defines SVC_Handler, PendSV_Handler and
 * SysTick_Handler itself, with the CMSIS names used by the vector table. */

#endif /* FREERTOS_CONFIG_H */
