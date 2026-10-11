/**
 * @file    bsp_uart.c
 * @brief   Interrupt-driven UART RX into ring buffers, blocking TX.
 *
 * RX path:  USART RXNE interrupt -> ring buffer -> callback (task notify).
 * The ISR is written at register level: it only moves bytes, so it is short
 * and never calls the HAL state machine.
 *
 * RS485 / Modbus: the USART "receiver timeout" (RTO) hardware counts idle
 * bit times after the last stop bit. Set to 3.5 characters (35 bits at
 * 8N1 + start/stop... 11 bits per char with parity, 10 without), it marks
 * exactly the Modbus RTU end-of-frame, with no software timer.
 * DE (driver enable) is also driven by the USART itself (HAL_RS485Ex_Init).
 */
#include "bsp.h"
#include "board.h"
#include "ring_buffer.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define RX_BUF_SIZE   512U

typedef struct {
    UART_HandleTypeDef h;
    ring_buffer_t      rx;
    uint8_t            rx_mem[RX_BUF_SIZE];
    bsp_uart_rx_cb_t   cb;
    SemaphoreHandle_t  tx_lock;
    volatile uint32_t  errors;
    bool               frame_mode;   /* RS485: callback on RTO, not per byte */
} uart_port_t;

static uart_port_t s_port[BSP_UART_COUNT];

static void pin_af(GPIO_TypeDef *port, uint32_t pin, uint8_t af, uint32_t pull)
{
    GPIO_InitTypeDef g = { 0 };
    g.Pin       = pin;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = pull;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = af;
    HAL_GPIO_Init(port, &g);
}

static void hw_setup(bsp_uart_t p, USART_TypeDef **inst, IRQn_Type *irq)
{
    switch (p) {
    case BSP_UART_CONSOLE:
        __HAL_RCC_USART2_CLK_ENABLE();
        pin_af(CONSOLE_TX_PORT, CONSOLE_TX_PIN, CONSOLE_AF, GPIO_NOPULL);
        pin_af(CONSOLE_RX_PORT, CONSOLE_RX_PIN, CONSOLE_AF, GPIO_PULLUP);
        *inst = CONSOLE_UART;
        *irq  = CONSOLE_IRQn;
        break;
    case BSP_UART_RS485:
        __HAL_RCC_USART1_CLK_ENABLE();
        pin_af(RS485_TX_PORT, RS485_TX_PIN, RS485_AF, GPIO_NOPULL);
        pin_af(RS485_RX_PORT, RS485_RX_PIN, RS485_AF, GPIO_PULLUP);
        pin_af(RS485_DE_PORT, RS485_DE_PIN, RS485_AF, GPIO_PULLDOWN);
        *inst = RS485_UART;
        *irq  = RS485_IRQn;
        break;
    case BSP_UART_LORA:
    default:
        __HAL_RCC_USART3_CLK_ENABLE();
        pin_af(LORA_TX_PORT, LORA_TX_PIN, LORA_AF, GPIO_NOPULL);
        pin_af(LORA_RX_PORT, LORA_RX_PIN, LORA_AF, GPIO_PULLUP);
        *inst = LORA_UART;
        *irq  = LORA_IRQn;
        break;
    }
}

void bsp_uart_init(bsp_uart_t p, uint32_t baud, bsp_uart_rx_cb_t cb)
{
    uart_port_t *u = &s_port[p];
    USART_TypeDef *inst;
    IRQn_Type irq;

    hw_setup(p, &inst, &irq);
    (void)rb_init(&u->rx, u->rx_mem, RX_BUF_SIZE);
    u->cb         = cb;
    u->tx_lock    = xSemaphoreCreateMutex();
    u->frame_mode = (p == BSP_UART_RS485);

    u->h.Instance            = inst;
    u->h.Init.BaudRate       = baud;
    u->h.Init.WordLength     = UART_WORDLENGTH_8B;
    u->h.Init.StopBits       = UART_STOPBITS_1;
    u->h.Init.Parity         = UART_PARITY_NONE;
    u->h.Init.Mode           = UART_MODE_TX_RX;
    u->h.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    u->h.Init.OverSampling   = UART_OVERSAMPLING_16;
    u->h.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    u->h.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    u->h.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (p == BSP_UART_RS485) {
        /* DE asserted/deasserted by hardware around each transmission. */
        (void)HAL_RS485Ex_Init(&u->h, UART_DE_POLARITY_HIGH, 0U, 0U);
        HAL_UART_ReceiverTimeout_Config(&u->h, 35U);   /* 3.5 chars x 10 bits */
        HAL_UART_EnableReceiverTimeout(&u->h);
        SET_BIT(inst->CR1, USART_CR1_RTOIE);
    } else {
        (void)HAL_UART_Init(&u->h);
    }

    /* RX interrupt + error interrupt (frame/noise/overrun). */
    SET_BIT(inst->CR1, USART_CR1_RXNEIE_RXFNEIE);
    SET_BIT(inst->CR3, USART_CR3_EIE);
    HAL_NVIC_SetPriority(irq, IRQ_PRIO_UART, 0U);
    HAL_NVIC_EnableIRQ(irq);
}

size_t bsp_uart_write(bsp_uart_t p, const void *data, size_t len)
{
    uart_port_t *u = &s_port[p];
    if (u->h.Instance == NULL || len == 0U) {
        return 0U;
    }
    bool locked = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
    if (locked) {
        xSemaphoreTake(u->tx_lock, portMAX_DELAY);
    }
    /* timeout: 2 ms per byte at 9600 baud is generous */
    HAL_StatusTypeDef st = HAL_UART_Transmit(&u->h, (const uint8_t *)data, (uint16_t)len, 10U + 2U * len);
    if (locked) {
        xSemaphoreGive(u->tx_lock);
    }
    return (st == HAL_OK) ? len : 0U;
}

size_t bsp_uart_read(bsp_uart_t p, void *data, size_t max)
{
    return rb_read(&s_port[p].rx, (uint8_t *)data, max);
}

size_t bsp_uart_available(bsp_uart_t p)
{
    return rb_count(&s_port[p].rx);
}

void bsp_uart_flush_rx(bsp_uart_t p)
{
    rb_clear(&s_port[p].rx);
}

uint32_t bsp_uart_errors(bsp_uart_t p)
{
    return s_port[p].errors;
}

/* ---- interrupt handling ---------------------------------------------- */
static void uart_isr(uart_port_t *u)
{
    USART_TypeDef *r = u->h.Instance;
    if (r == NULL) {
        return;
    }
    uint32_t isr = r->ISR;
    bool notify = false;

    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        r->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        u->errors++;
    }
    while (r->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t b = (uint8_t)r->RDR;            /* reading RDR clears RXNE */
        if (!rb_put(&u->rx, b)) {
            u->errors++;                        /* software overflow */
        }
        notify = !u->frame_mode;
    }
    if (u->frame_mode && (isr & USART_ISR_RTOF)) {
        r->ICR = USART_ICR_RTOCF;
        notify = true;                          /* end of Modbus frame */
    }
    if (notify && u->cb != NULL) {
        u->cb((bsp_uart_t)(u - s_port));
    }
}

void USART2_LPUART2_IRQHandler(void)       { uart_isr(&s_port[BSP_UART_CONSOLE]); }
void USART1_IRQHandler(void)               { uart_isr(&s_port[BSP_UART_RS485]); }
void USART3_4_5_6_LPUART1_IRQHandler(void) { uart_isr(&s_port[BSP_UART_LORA]); }
