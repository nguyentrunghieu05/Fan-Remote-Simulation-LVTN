/**
 * @file    board.h
 * @brief   Pin map of the target: NUCLEO-G0B1RE (STM32G0B1RET6) + modules.
 *
 * This is the ONLY file that names pins. Porting to a custom PCB means
 * editing this file, nothing above the BSP changes.
 *
 *  Function             Peripheral   Pins (AF)              Nucleo connector
 *  -------------------  -----------  ---------------------  ----------------
 *  Console / CLI        USART2       PA2 TX, PA3 RX (AF1)   ST-LINK VCP (USB)
 *  RS485 Modbus slave   USART1       PA9 TX, PA10 RX,       D8, D2,
 *                                    PA12 DE (AF1)          CN10-12
 *  LoRa module (E220)   USART3       PC10 TX, PC11 RX (AF0) CN7-1, CN7-2
 *                       GPIO         PC2 M0, PC3 M1         CN7-35, CN7-37
 *  SPI flash W25Q64     SPI2         PB13 SCK, PB14 MISO,   CN10-30/28/26
 *                                    PB15 MOSI (AF0)
 *                       GPIO         PB12 CS                CN10-16
 *  SHT31 + SSD1306 OLED I2C1         PB8 SCL, PB9 SDA (AF6) D15, D14
 *  Fan PWM 25 kHz       TIM1_CH1     PA8 (AF2)              D7
 *  Fan 0-10 V (via op-amp x3) DAC1_OUT1 PA4                 A2
 *  Fan tachometer       TIM2_CH1     PA0 (AF2)              A0
 *  Speed knob (pot)     ADC1_IN1     PA1                    A1
 *  Alarm relay          GPIO         PC0                    A5 (CN8-6)
 *  Status LED           GPIO         PA5                    LD4 (on board)
 *  User button          GPIO         PC13                   B1  (on board)
 *
 * !! Check every pin against the STM32G0B1 datasheet (table "Alternate
 *    functions") and the Nucleo user manual UM2324 before wiring.
 */
#ifndef BOARD_H
#define BOARD_H

#include "stm32g0xx_hal.h"

/* ---- Console (ST-LINK virtual COM port) ---- */
#define CONSOLE_UART            USART2
#define CONSOLE_BAUD            115200U
#define CONSOLE_TX_PORT         GPIOA
#define CONSOLE_TX_PIN          GPIO_PIN_2
#define CONSOLE_RX_PORT         GPIOA
#define CONSOLE_RX_PIN          GPIO_PIN_3
#define CONSOLE_AF              GPIO_AF1_USART2
#define CONSOLE_IRQn            USART2_LPUART2_IRQn

/* ---- RS485 (MAX485 / SP3485 transceiver, DE+RE tied together) ---- */
#define RS485_UART              USART1
#define RS485_BAUD              9600U
#define RS485_TX_PORT           GPIOA
#define RS485_TX_PIN            GPIO_PIN_9
#define RS485_RX_PORT           GPIOA
#define RS485_RX_PIN            GPIO_PIN_10
#define RS485_DE_PORT           GPIOA
#define RS485_DE_PIN            GPIO_PIN_12
#define RS485_AF                GPIO_AF1_USART1
#define RS485_IRQn              USART1_IRQn

/* ---- LoRa UART module (Ebyte E220/E32, transparent mode) ---- */
#define LORA_UART               USART3
#define LORA_BAUD               9600U
#define LORA_TX_PORT            GPIOC
#define LORA_TX_PIN             GPIO_PIN_10
#define LORA_RX_PORT            GPIOC
#define LORA_RX_PIN             GPIO_PIN_11
#define LORA_AF                 GPIO_AF0_USART3
#define LORA_IRQn               USART3_4_5_6_LPUART1_IRQn
#define LORA_M0_PORT            GPIOC
#define LORA_M0_PIN             GPIO_PIN_2
#define LORA_M1_PORT            GPIOC
#define LORA_M1_PIN             GPIO_PIN_3

/* ---- SPI flash ---- */
#define FLASH_SPI               SPI2
#define FLASH_SPI_AF            GPIO_AF0_SPI2
#define FLASH_SCK_PORT          GPIOB
#define FLASH_SCK_PIN           GPIO_PIN_13
#define FLASH_MISO_PORT         GPIOB
#define FLASH_MISO_PIN          GPIO_PIN_14
#define FLASH_MOSI_PORT         GPIOB
#define FLASH_MOSI_PIN          GPIO_PIN_15
#define FLASH_CS_PORT           GPIOB
#define FLASH_CS_PIN            GPIO_PIN_12

/* ---- I2C sensor + display bus ---- */
#define SENSOR_I2C              I2C1
#define SENSOR_I2C_AF           GPIO_AF6_I2C1
#define SENSOR_SCL_PORT         GPIOB
#define SENSOR_SCL_PIN          GPIO_PIN_8
#define SENSOR_SDA_PORT         GPIOB
#define SENSOR_SDA_PIN          GPIO_PIN_9
/* TIMINGR for 400 kHz with I2C kernel clock = PCLK = 64 MHz:
 * PRESC=7 (125 ns), SCLDEL=3, SDADEL=0, SCLH=5, SCLL=11 */
#define SENSOR_I2C_TIMING       0x7030050BUL

/* ---- Fan outputs / inputs ---- */
#define FAN_PWM_TIM             TIM1
#define FAN_PWM_CHANNEL         TIM_CHANNEL_1
#define FAN_PWM_PORT            GPIOA
#define FAN_PWM_PIN             GPIO_PIN_8
#define FAN_PWM_AF              GPIO_AF2_TIM1
#define FAN_PWM_FREQ_HZ         25000U          /* Intel 4-wire fan spec */

#define FAN_TACH_TIM            TIM2
#define FAN_TACH_PORT           GPIOA
#define FAN_TACH_PIN            GPIO_PIN_0
#define FAN_TACH_AF             GPIO_AF2_TIM2
#define FAN_TACH_IRQn           TIM2_IRQn
#define FAN_TACH_PULSES_PER_REV 2U

#define FAN_DAC_PORT            GPIOA
#define FAN_DAC_PIN             GPIO_PIN_4

#define POT_ADC_PORT            GPIOA
#define POT_ADC_PIN             GPIO_PIN_1
#define POT_ADC_CHANNEL         ADC_CHANNEL_1

/* ---- Simple I/O ---- */
#define RELAY_PORT              GPIOC
#define RELAY_PIN               GPIO_PIN_0
#define LED_PORT                GPIOA
#define LED_PIN                 GPIO_PIN_5
#define BUTTON_PORT             GPIOC
#define BUTTON_PIN              GPIO_PIN_13
#define BUTTON_PRESSED_LEVEL    GPIO_PIN_RESET   /* flip if B1 reads high when pressed */

/* ---- Storage layout on the external flash ---- */
#define STORAGE_CFG_A_ADDR      0x000000UL       /* sector 0: config copy A */
#define STORAGE_CFG_B_ADDR      0x001000UL       /* sector 1: config copy B */
#define STORAGE_LOG_ADDR        0x010000UL       /* data log from 64 KB ... */
#define STORAGE_LOG_SECTORS_MAX 2032U            /* ... to the end of 8 MB  */

/* ---- Interrupt priorities (Cortex-M0+: 0 = highest .. 3 = lowest) ----
 * Every ISR that calls a FreeRTOS "FromISR" API must have a priority
 * numerically >= configMAX_SYSCALL priority; on M0+ the port masks all
 * interrupts in critical sections, so any level works, we keep the UARTs
 * above the timer to never lose a byte. */
#define IRQ_PRIO_UART           1U
#define IRQ_PRIO_TACH           2U

#endif /* BOARD_H */
