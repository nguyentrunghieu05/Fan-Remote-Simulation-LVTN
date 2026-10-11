/**
 * @file    bsp_bus.c
 * @brief   SPI2 (external flash + MCP2515 CAN controller) and I2C1 (sensor + display) buses.
 *
 * Each bus has a mutex, so two tasks (e.g. the sensor task reading the
 * SHT31 and the display task refreshing the OLED) can share it safely.
 * The structs filled here are the hardware-independent spi_bus_t /
 * i2c_bus_t that the device drivers use.
 */
#include "bsp.h"
#include "board.h"
#include "err.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SPI_HandleTypeDef s_spi;
static I2C_HandleTypeDef s_i2c;
static SemaphoreHandle_t s_spi_lock;
static SemaphoreHandle_t s_i2c_lock;

#define BUS_TIMEOUT_MS  50U

static bool rtos_running(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/* ---------------------------------------------------------------- SPI */
/* One SPI2 bus, one entry per slave: its chip select and its clock. */
typedef struct {
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;
    uint32_t      prescaler;    /* SPI_BAUDRATEPRESCALER_x = CR1.BR bits */
} spi_dev_t;

static const spi_dev_t s_flash_dev = { FLASH_CS_PORT, FLASH_CS_PIN, FLASH_SPI_PRESCALER };
static const spi_dev_t s_can_dev   = { CAN_CS_PORT, CAN_CS_PIN, CAN_SPI_PRESCALER };

static void spi_select(void *ctx, bool active)
{
    const spi_dev_t *dev = ctx;
    if (active) {
        if (rtos_running()) {
            xSemaphoreTake(s_spi_lock, portMAX_DELAY);   /* lock for the whole CS window */
        }
        if ((FLASH_SPI->CR1 & SPI_CR1_BR) != dev->prescaler) {
            __HAL_SPI_DISABLE(&s_spi);                   /* BR may only change while SPE = 0; */
            MODIFY_REG(FLASH_SPI->CR1, SPI_CR1_BR, dev->prescaler);   /* HAL re-enables it */
        }
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
        if (rtos_running()) {
            xSemaphoreGive(s_spi_lock);
        }
    }
}

static int spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx;
    HAL_StatusTypeDef st;
    uint32_t timeout = BUS_TIMEOUT_MS + (uint32_t)(len / 1000U);
    if (tx != NULL && rx != NULL) {
        st = HAL_SPI_TransmitReceive(&s_spi, (uint8_t *)tx, rx, (uint16_t)len, timeout);
    } else if (tx != NULL) {
        st = HAL_SPI_Transmit(&s_spi, (uint8_t *)tx, (uint16_t)len, timeout);
    } else {
        st = HAL_SPI_Receive(&s_spi, rx, (uint16_t)len, timeout);   /* clocks out dummy bytes */
    }
    return (st == HAL_OK) ? ERR_OK : (st == HAL_TIMEOUT ? ERR_TIMEOUT : ERR_IO);
}

void bsp_spi_init(spi_bus_t *bus)
{
    GPIO_InitTypeDef g = { 0 };
    __HAL_RCC_SPI2_CLK_ENABLE();
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = FLASH_SPI_AF;
    g.Pin = FLASH_SCK_PIN;  HAL_GPIO_Init(FLASH_SCK_PORT, &g);
    g.Pin = FLASH_MOSI_PIN; HAL_GPIO_Init(FLASH_MOSI_PORT, &g);
    g.Pull = GPIO_PULLUP;                               /* MISO reads 0xFF if no chip */
    g.Pin = FLASH_MISO_PIN; HAL_GPIO_Init(FLASH_MISO_PORT, &g);

    s_spi.Instance               = FLASH_SPI;
    s_spi.Init.Mode              = SPI_MODE_MASTER;
    s_spi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_spi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_spi.Init.CLKPolarity       = SPI_POLARITY_LOW;    /* SPI mode 0 */
    s_spi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_spi.Init.NSS               = SPI_NSS_SOFT;        /* CS is a plain GPIO */
    s_spi.Init.BaudRatePrescaler = FLASH_SPI_PRESCALER;
    s_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_spi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    (void)HAL_SPI_Init(&s_spi);

    s_spi_lock    = xSemaphoreCreateMutex();
    bus->transfer = spi_transfer;
    bus->select   = spi_select;
    bus->ctx      = (void *)&s_flash_dev;
}

void bsp_spi_can_bus(spi_bus_t *bus)
{
    bus->transfer = spi_transfer;
    bus->select   = spi_select;
    bus->ctx      = (void *)&s_can_dev;
}

/* ---------------------------------------------------------------- I2C */
static int i2c_status(HAL_StatusTypeDef st)
{
    if (st == HAL_OK) {
        return ERR_OK;
    }
    if (HAL_I2C_GetError(&s_i2c) & HAL_I2C_ERROR_AF) {
        return ERR_IO;                                  /* NACK: device absent or busy */
    }
    if (HAL_I2C_GetError(&s_i2c) & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO) || st == HAL_TIMEOUT) {
        bsp_i2c_recover();                              /* bus may be stuck */
    }
    return (st == HAL_TIMEOUT) ? ERR_TIMEOUT : ERR_IO;
}

static int i2c_write(void *ctx, uint8_t addr, const uint8_t *data, size_t len)
{
    (void)ctx;
    if (rtos_running()) { xSemaphoreTake(s_i2c_lock, portMAX_DELAY); }
    int rc = i2c_status(HAL_I2C_Master_Transmit(&s_i2c, (uint16_t)(addr << 1), (uint8_t *)data,
                                                (uint16_t)len, BUS_TIMEOUT_MS));
    if (rtos_running()) { xSemaphoreGive(s_i2c_lock); }
    return rc;
}

static int i2c_read(void *ctx, uint8_t addr, uint8_t *data, size_t len)
{
    (void)ctx;
    if (rtos_running()) { xSemaphoreTake(s_i2c_lock, portMAX_DELAY); }
    int rc = i2c_status(HAL_I2C_Master_Receive(&s_i2c, (uint16_t)(addr << 1), data,
                                               (uint16_t)len, BUS_TIMEOUT_MS));
    if (rtos_running()) { xSemaphoreGive(s_i2c_lock); }
    return rc;
}

static void i2c_pins_af(void)
{
    GPIO_InitTypeDef g = { 0 };
    g.Mode      = GPIO_MODE_AF_OD;                      /* open drain, pull-ups on the module */
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = SENSOR_I2C_AF;
    g.Pin = SENSOR_SCL_PIN; HAL_GPIO_Init(SENSOR_SCL_PORT, &g);
    g.Pin = SENSOR_SDA_PIN; HAL_GPIO_Init(SENSOR_SDA_PORT, &g);
}

void bsp_i2c_recover(void)
{
    /* Take the pins back as GPIO and clock SCL until the slave releases SDA,
     * then generate a STOP condition (I2C spec, section 3.1.16). */
    GPIO_InitTypeDef g = { 0 };
    (void)HAL_I2C_DeInit(&s_i2c);
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = SENSOR_SCL_PIN; HAL_GPIO_Init(SENSOR_SCL_PORT, &g);
    g.Pin = SENSOR_SDA_PIN; HAL_GPIO_Init(SENSOR_SDA_PORT, &g);
    HAL_GPIO_WritePin(SENSOR_SDA_PORT, SENSOR_SDA_PIN, GPIO_PIN_SET);
    for (int i = 0; i < 9 && HAL_GPIO_ReadPin(SENSOR_SDA_PORT, SENSOR_SDA_PIN) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(SENSOR_SCL_PORT, SENSOR_SCL_PIN, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 100; d++) { }
        HAL_GPIO_WritePin(SENSOR_SCL_PORT, SENSOR_SCL_PIN, GPIO_PIN_SET);
        for (volatile int d = 0; d < 100; d++) { }
    }
    HAL_GPIO_WritePin(SENSOR_SDA_PORT, SENSOR_SDA_PIN, GPIO_PIN_RESET);   /* STOP: SDA low->high */
    for (volatile int d = 0; d < 100; d++) { }
    HAL_GPIO_WritePin(SENSOR_SDA_PORT, SENSOR_SDA_PIN, GPIO_PIN_SET);
    i2c_pins_af();
    (void)HAL_I2C_Init(&s_i2c);
}

bool bsp_i2c_probe(uint8_t addr)
{
    if (rtos_running()) { xSemaphoreTake(s_i2c_lock, portMAX_DELAY); }
    bool ok = HAL_I2C_IsDeviceReady(&s_i2c, (uint16_t)(addr << 1), 1U, 5U) == HAL_OK;
    if (rtos_running()) { xSemaphoreGive(s_i2c_lock); }
    return ok;
}

void bsp_i2c_init(i2c_bus_t *bus)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    i2c_pins_af();
    s_i2c.Instance              = SENSOR_I2C;
    s_i2c.Init.Timing           = SENSOR_I2C_TIMING;
    s_i2c.Init.OwnAddress1      = 0U;
    s_i2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    s_i2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    s_i2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    s_i2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&s_i2c);
    (void)HAL_I2CEx_ConfigAnalogFilter(&s_i2c, I2C_ANALOGFILTER_ENABLE);

    s_i2c_lock = xSemaphoreCreateMutex();
    bus->write = i2c_write;
    bus->read  = i2c_read;
    bus->ctx   = NULL;
}
