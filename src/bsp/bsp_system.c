/**
 * @file    bsp_system.c
 * @brief   Clock tree, GPIO, delays, reset cause.
 */
#include "bsp.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

static const char *s_reset_cause = "?";
static TIM_HandleTypeDef s_tick_tim;

/* HAL time base on TIM6 (1 kHz), because SysTick is used by FreeRTOS.
 * Overrides the weak HAL_InitTick(); called by HAL_Init() and again by
 * HAL_RCC_ClockConfig() when the clock changes. */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    s_tick_tim.Instance           = TIM6;
    s_tick_tim.Init.Prescaler     = (HAL_RCC_GetPCLK1Freq() / 1000000U) - 1U;   /* 1 MHz */
    s_tick_tim.Init.CounterMode   = TIM_COUNTERMODE_UP;
    s_tick_tim.Init.Period        = 1000U - 1U;                                  /* 1 ms  */
    s_tick_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&s_tick_tim) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, TickPriority, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);
    uwTickPrio = TickPriority;
    return HAL_TIM_Base_Start_IT(&s_tick_tim);
}

void TIM6_DAC_LPTIM1_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR = ~TIM_SR_UIF;
        HAL_IncTick();
    }
}

void bsp_clock_init(void)
{
    RCC_OscInitTypeDef osc = { 0 };
    RCC_ClkInitTypeDef clk = { 0 };

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSI16 -> PLL: 16 MHz / M(1) * N(8) / R(2) = 64 MHz (G0 maximum) */
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSIDiv              = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = RCC_PLLM_DIV1;
    osc.PLL.PLLN            = 8;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = RCC_PLLQ_DIV2;
    osc.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        for (;;) { }
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
        for (;;) { }
    }

    /* Remember why we restarted, then clear the flags. */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        s_reset_cause = "IWDG";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
        s_reset_cause = "SOFTWARE";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PWRRST)) {
        s_reset_cause = "POWER";
    } else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
        s_reset_cause = "PIN";
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

const char *bsp_reset_cause(void)
{
    return s_reset_cause;
}

static void out_pin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState init)
{
    GPIO_InitTypeDef g = { 0 };
    HAL_GPIO_WritePin(port, pin, init);
    g.Pin   = pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
}

void bsp_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    out_pin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    out_pin(RELAY_PORT, RELAY_PIN, GPIO_PIN_RESET);
    out_pin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);        /* deselected */
    out_pin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);        /* M0=M1=0: transparent */
    out_pin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);

    GPIO_InitTypeDef g = { 0 };
    g.Pin  = BUTTON_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;                                      /* Nucleo has its own pull resistor */
    HAL_GPIO_Init(BUTTON_PORT, &g);
}

void bsp_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms) > 0 ? pdMS_TO_TICKS(ms) : 1);
    } else {
        HAL_Delay(ms);
    }
}

uint32_t bsp_millis(void)
{
    return HAL_GetTick();
}

void bsp_reset(void)
{
    NVIC_SystemReset();
}

void bsp_led_set(bool on)           { HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }
void bsp_led_toggle(void)           { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); }
bool bsp_button_pressed(void)       { return HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == BUTTON_PRESSED_LEVEL; }
void bsp_relay_set(bool on)         { HAL_GPIO_WritePin(RELAY_PORT, RELAY_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void bsp_lora_set_mode(uint8_t m0, uint8_t m1)
{
    HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
