/**
 * @file    bsp.h
 * @brief   Board Support Package: the only layer that calls the STM32 HAL.
 *
 * Everything above (services, app) uses these functions and the generic
 * bus structs from bus.h, never a register or a HAL handle.
 */
#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bus.h"

/* ---- system ---- */
void     bsp_clock_init(void);          /* HSI16 -> PLL -> 64 MHz */
void     bsp_gpio_init(void);
void     bsp_delay_ms(uint32_t ms);     /* vTaskDelay once the scheduler runs */
uint32_t bsp_millis(void);
void     bsp_reset(void);
const char *bsp_reset_cause(void);      /* read once at boot: "POWER", "IWDG"... */

/* ---- simple I/O ---- */
void bsp_led_set(bool on);
void bsp_led_toggle(void);
bool bsp_button_pressed(void);
void bsp_relay_set(bool on);
void bsp_lora_set_mode(uint8_t m0, uint8_t m1);

/* ---- UART ---- */
typedef enum {
    BSP_UART_CONSOLE = 0,
    BSP_UART_RS485,
    BSP_UART_LORA,
    BSP_UART_COUNT
} bsp_uart_t;

/** Called from the UART interrupt: a byte arrived (console/LoRa) or a
 *  Modbus frame ended (RS485, 3.5-character silence). */
typedef void (*bsp_uart_rx_cb_t)(bsp_uart_t port);

void   bsp_uart_init(bsp_uart_t port, uint32_t baud, bsp_uart_rx_cb_t cb);
size_t bsp_uart_write(bsp_uart_t port, const void *data, size_t len);
size_t bsp_uart_read(bsp_uart_t port, void *data, size_t max);
size_t bsp_uart_available(bsp_uart_t port);
void   bsp_uart_flush_rx(bsp_uart_t port);
uint32_t bsp_uart_errors(bsp_uart_t port);

/* ---- SPI flash bus / I2C sensor bus (thread safe) ---- */
void bsp_spi_init(spi_bus_t *flash_bus);
/** Same SPI2 bus, MCP2515 chip select and 8 MHz clock. Call after bsp_spi_init(). */
void bsp_spi_can_bus(spi_bus_t *can_bus);
void bsp_i2c_init(i2c_bus_t *sensor_bus);
/** Free a bus stuck by a slave holding SDA low (9 clock pulses + STOP). */
void bsp_i2c_recover(void);
/** Probe one address (used by the "i2c scan" CLI command). */
bool bsp_i2c_probe(uint8_t addr);

/* ---- analog / timers ---- */
void     bsp_adc_init(void);
uint16_t bsp_adc_read_pot(void);        /* 0..4095 */
void     bsp_fan_out_init(void);
void     bsp_fan_out_set(float percent);/* PWM duty + DAC voltage together */
void     bsp_tach_init(void);
uint16_t bsp_tach_rpm(void);

/* ---- RTC / watchdog ---- */
void     bsp_rtc_init(void);
uint32_t bsp_rtc_get_unix(void);
void     bsp_rtc_set_unix(uint32_t t);
bool     bsp_rtc_is_set(void);
void     bsp_iwdg_init(uint32_t timeout_ms);
void     bsp_iwdg_refresh(void);

#endif /* BSP_H */
