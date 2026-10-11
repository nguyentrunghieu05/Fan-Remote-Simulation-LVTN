/**
 * @file    interrupts.c
 * @brief   Core exception handlers and FreeRTOS hooks.
 *          (Peripheral IRQ handlers live next to their driver in the BSP.)
 */
#include "stm32g0xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* SysTick, SVC and PendSV belong to the FreeRTOS port (port.c / portasm.c).
 * The HAL time base runs on TIM6 instead (see HAL_InitTick in bsp_system.c),
 * so HAL timeouts keep working even inside critical sections of the kernel. */

void NMI_Handler(void)
{
    for (;;) { }
}

/* On a hard fault, stop here with the debugger (registers are on the
 * stack). Without a debugger the IWDG resets the MCU after 2 s. */
void HardFault_Handler(void)
{
    __asm volatile ("bkpt #0");
    for (;;) { }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;                     /* inspect 'name' in the debugger */
    __disable_irq();
    for (;;) { }
}

void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    for (;;) { }
}
