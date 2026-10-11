/**
 * @file    bsp_analog.c
 * @brief   ADC (speed knob, VDD via VREFINT, 12 V supply), fan outputs (PWM + DAC) and tachometer.
 *
 * Fan output: the same duty is sent two ways so the board drives both fan
 * types used in industry:
 *   - 25 kHz PWM on TIM1_CH1 -> 4-wire PC/EC fans (Intel spec)
 *   - DAC 0..3.3 V -> op-amp x3 -> 0..10 V input of an inverter / EC motor
 *
 * Tachometer: TIM2 runs free at 1 MHz (32-bit). Each falling edge of the
 * fan tach signal is captured; the period between two edges gives the
 * speed. The same 1 MHz counter is the time base for FreeRTOS run-time
 * statistics (CPU load per task).
 */
#include "bsp.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

static ADC_HandleTypeDef s_adc;
static TIM_HandleTypeDef s_pwm;
static DAC_HandleTypeDef s_dac;
static TIM_HandleTypeDef s_tach;

/* ------------------------------------------------------------------ ADC */
void bsp_adc_init(void)
{
    GPIO_InitTypeDef g = { 0 };
    ADC_ChannelConfTypeDef ch = { 0 };

    __HAL_RCC_ADC_CLK_ENABLE();
    g.Pin  = POT_ADC_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(POT_ADC_PORT, &g);
    g.Pin  = VBUS_ADC_PIN;
    HAL_GPIO_Init(VBUS_ADC_PORT, &g);

    s_adc.Instance                   = ADC1;
    s_adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    s_adc.Init.Resolution            = ADC_RESOLUTION_12B;
    s_adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_adc.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    s_adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    s_adc.Init.LowPowerAutoWait      = DISABLE;
    s_adc.Init.ContinuousConvMode    = DISABLE;
    s_adc.Init.NbrOfConversion       = 1;
    s_adc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    s_adc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    s_adc.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_79CYCLES_5;  /* pot is high impedance */
    s_adc.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_160CYCLES_5; /* VREFINT needs >= 4 us: 5 us at 32 MHz */
    s_adc.Init.OversamplingMode      = ENABLE;                     /* 16x hardware average */
    s_adc.Init.Oversampling.Ratio         = ADC_OVERSAMPLING_RATIO_16;
    s_adc.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
    s_adc.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    (void)HAL_ADC_Init(&s_adc);

    ch.Channel      = POT_ADC_CHANNEL;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    (void)HAL_ADC_ConfigChannel(&s_adc, &ch);
    (void)HAL_ADCEx_Calibration_Start(&s_adc);
}

/* One conversion of one channel: rank 1 of the sequencer is switched to it. */
static uint16_t adc_read(uint32_t channel, uint32_t sampling)
{
    ADC_ChannelConfTypeDef ch = { 0 };
    ch.Channel      = channel;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = sampling;
    (void)HAL_ADC_ConfigChannel(&s_adc, &ch);

    uint16_t v = 0U;
    if (HAL_ADC_Start(&s_adc) == HAL_OK &&
        HAL_ADC_PollForConversion(&s_adc, 5U) == HAL_OK) {
        v = (uint16_t)HAL_ADC_GetValue(&s_adc);
    }
    (void)HAL_ADC_Stop(&s_adc);
    return v;
}

uint16_t bsp_adc_read_pot(void)
{
    return adc_read(POT_ADC_CHANNEL, ADC_SAMPLINGTIME_COMMON_1);
}

uint16_t bsp_adc_vdd_mv(void)
{
    uint16_t raw = adc_read(ADC_CHANNEL_VREFINT, ADC_SAMPLINGTIME_COMMON_2);
    if (raw == 0U) {
        return 0U;
    }
    /* VDDA = 3.0 V * VREFINT_CAL / raw (VREFINT_CAL measured at 3.0 V in the factory) */
    return (uint16_t)__LL_ADC_CALC_VREFANALOG_VOLTAGE(raw, LL_ADC_RESOLUTION_12B);
}

uint32_t bsp_adc_vbus_mv(void)
{
    uint32_t vdd = bsp_adc_vdd_mv();
    uint32_t raw = adc_read(VBUS_ADC_CHANNEL, ADC_SAMPLINGTIME_COMMON_1);
    uint32_t pin_mv = raw * vdd / 4095U;
    return pin_mv * (VBUS_DIV_R1_OHM + VBUS_DIV_R2_OHM) / VBUS_DIV_R2_OHM;
}

/* ------------------------------------------------------- fan outputs */
static uint32_t s_pwm_period;

void bsp_fan_out_init(void)
{
    GPIO_InitTypeDef g = { 0 };
    TIM_OC_InitTypeDef oc = { 0 };
    DAC_ChannelConfTypeDef dc = { 0 };

    /* PWM: 64 MHz / 25 kHz = 2560 counts -> 0.04 % resolution */
    __HAL_RCC_TIM1_CLK_ENABLE();
    g.Pin       = FAN_PWM_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = FAN_PWM_AF;
    HAL_GPIO_Init(FAN_PWM_PORT, &g);

    s_pwm_period = (SystemCoreClock / FAN_PWM_FREQ_HZ) - 1U;
    s_pwm.Instance               = FAN_PWM_TIM;
    s_pwm.Init.Prescaler         = 0U;
    s_pwm.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_pwm.Init.Period            = s_pwm_period;
    s_pwm.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_pwm.Init.RepetitionCounter = 0U;
    s_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    (void)HAL_TIM_PWM_Init(&s_pwm);
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    (void)HAL_TIM_PWM_ConfigChannel(&s_pwm, &oc, FAN_PWM_CHANNEL);
    (void)HAL_TIM_PWM_Start(&s_pwm, FAN_PWM_CHANNEL);   /* also sets MOE on TIM1 */

    /* DAC */
    __HAL_RCC_DAC1_CLK_ENABLE();
    g.Pin  = FAN_DAC_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(FAN_DAC_PORT, &g);
    s_dac.Instance = DAC1;
    (void)HAL_DAC_Init(&s_dac);
    dc.DAC_SampleAndHold       = DAC_SAMPLEANDHOLD_DISABLE;
    dc.DAC_Trigger             = DAC_TRIGGER_NONE;
    dc.DAC_OutputBuffer        = DAC_OUTPUTBUFFER_ENABLE;
    dc.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    dc.DAC_UserTrimming        = DAC_TRIMMING_FACTORY;
    (void)HAL_DAC_ConfigChannel(&s_dac, &dc, DAC_CHANNEL_1);
    (void)HAL_DAC_SetValue(&s_dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0U);
    (void)HAL_DAC_Start(&s_dac, DAC_CHANNEL_1);
}

void bsp_fan_out_set(float percent)
{
    if (percent < 0.0f)   { percent = 0.0f; }
    if (percent > 100.0f) { percent = 100.0f; }
    uint32_t ccr = (uint32_t)((s_pwm_period + 1U) * percent / 100.0f + 0.5f);
    __HAL_TIM_SET_COMPARE(&s_pwm, FAN_PWM_CHANNEL, ccr);
    uint32_t code = (uint32_t)(4095.0f * percent / 100.0f + 0.5f);
    (void)HAL_DAC_SetValue(&s_dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code);
}

/* -------------------------------------------------------- tachometer */
static volatile uint32_t s_tach_last;      /* counter value at last edge  */
static volatile uint32_t s_tach_period;    /* us between two edges        */
static volatile uint32_t s_tach_last_ms;   /* system time of the last edge */

void bsp_tach_init(void)
{
    GPIO_InitTypeDef g = { 0 };
    TIM_IC_InitTypeDef ic = { 0 };

    if (s_tach.Instance != NULL) {
        return;                              /* already started (run-time stats) */
    }
    __HAL_RCC_TIM2_CLK_ENABLE();
    g.Pin       = FAN_TACH_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;               /* tach is open collector */
    g.Alternate = FAN_TACH_AF;
    HAL_GPIO_Init(FAN_TACH_PORT, &g);

    s_tach.Instance           = FAN_TACH_TIM;
    s_tach.Init.Prescaler     = (SystemCoreClock / 1000000U) - 1U;   /* 1 MHz */
    s_tach.Init.CounterMode   = TIM_COUNTERMODE_UP;
    s_tach.Init.Period        = 0xFFFFFFFFUL;
    s_tach.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    (void)HAL_TIM_IC_Init(&s_tach);
    ic.ICPolarity  = TIM_ICPOLARITY_FALLING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 0x0FU;                  /* strongest digital filter: tach is noisy */
    (void)HAL_TIM_IC_ConfigChannel(&s_tach, &ic, TIM_CHANNEL_1);
    HAL_NVIC_SetPriority(FAN_TACH_IRQn, IRQ_PRIO_TACH, 0U);
    HAL_NVIC_EnableIRQ(FAN_TACH_IRQn);
    (void)HAL_TIM_IC_Start_IT(&s_tach, TIM_CHANNEL_1);
}

void TIM2_IRQHandler(void)
{
    if (FAN_TACH_TIM->SR & TIM_SR_CC1IF) {
        uint32_t now = FAN_TACH_TIM->CCR1;   /* reading CCR1 clears CC1IF */
        uint32_t p   = now - s_tach_last;    /* unsigned math handles wrap */
        s_tach_last  = now;
        if (p > 500U) {                      /* > 2 kHz edge rate = noise, ignore */
            s_tach_period  = p;
            s_tach_last_ms = HAL_GetTick();
        }
    }
    FAN_TACH_TIM->SR = 0U;
}

uint16_t bsp_tach_rpm(void)
{
    uint32_t period = s_tach_period;
    if (period == 0U || HAL_GetTick() - s_tach_last_ms > 1000U) {
        return 0U;                           /* no edge for 1 s: stopped */
    }
    uint32_t rpm = 60000000UL / (period * FAN_TACH_PULSES_PER_REV);
    return (rpm > 0xFFFFU) ? 0xFFFFU : (uint16_t)rpm;
}

/* FreeRTOS run-time statistics: 1 us resolution from TIM2. */
void rtstats_timer_init(void)
{
    bsp_tach_init();
}

uint32_t rtstats_timer_get(void)
{
    return FAN_TACH_TIM->CNT;
}
