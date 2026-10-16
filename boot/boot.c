/**
 * @file    boot.c
 * @brief   Bootloader: installs a verified image from the external W25Q64.
 *
 * Runs from the first 32 KB of flash, before the application, without RTOS.
 * At every reset:
 *   1. read the update state (config_store A/B on the W25Q64) and check the
 *      NEW slot, the BACKUP slot and the application vector table;
 *   2. let fw_boot_decide() choose: run, install NEW, or roll back to BACKUP;
 *   3. install = copy the running application to BACKUP, then NEW into the
 *      internal flash, then check the CRC of what was programmed;
 *   4. save the new state only after the copy succeeded;
 *   5. start the IWDG (BOOT_IWDG_MS) and jump.
 * A power cut at any point leaves the NEW slot untouched and the state as it
 * was, so the same step is simply done again at the next reset.
 *
 * Reset flags in RCC->CSR are left as they are: the application reads them
 * to report the reset cause (IWDG, PIN, ...).
 */
#include "stm32g0xx_hal.h"
#include "board.h"
#include "err.h"
#include "crc.h"
#include "w25qxx.h"
#include "config_store.h"
#include "fw_image.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define BOOT_VERSION   "1.0"
#define RAM_START      0x20000000UL
#define RAM_END        (RAM_START + 144UL * 1024UL)
#define PAGE           FLASH_PAGE_SIZE           /* 2 KB */
#define BOOT_IWDG_MS   8000U                     /* until the application sets its own */

static UART_HandleTypeDef s_uart;
static SPI_HandleTypeDef  s_spi;
static w25qxx_t           s_w25;
static flash_dev_t        s_flash;
static uint8_t            s_buf[PAGE];

void SysTick_Handler(void) { HAL_IncTick(); }

/* newlib stubs: the bootloader has no files (vsnprintf only). */
int _write(int fd, const char *buf, int len) { (void)fd; (void)buf; return len; }
int _read(int fd, char *buf, int len)        { (void)fd; (void)buf; (void)len; return 0; }
int _close(int fd)                           { (void)fd; return -1; }
int _lseek(int fd, int off, int dir)         { (void)fd; (void)off; (void)dir; return 0; }

/* ------------------------------------------------------------ console */
static void say(const char *fmt, ...)
{
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof line - 2, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof line - 3) {
        n = (int)sizeof line - 3;
    }
    line[n++] = '\r';
    line[n++] = '\n';
    (void)HAL_UART_Transmit(&s_uart, (uint8_t *)line, (uint16_t)n, 100U);
}

static void uart_init(void)
{
    GPIO_InitTypeDef g = { 0 };
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = CONSOLE_AF;
    g.Pin = CONSOLE_TX_PIN; HAL_GPIO_Init(CONSOLE_TX_PORT, &g);
    g.Pin = CONSOLE_RX_PIN; HAL_GPIO_Init(CONSOLE_RX_PORT, &g);
    s_uart.Instance          = CONSOLE_UART;
    s_uart.Init.BaudRate     = CONSOLE_BAUD;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    (void)HAL_UART_Init(&s_uart);
}

/* ------------------------------------------------- SPI2 + W25Q64 */
static void spi_select(void *ctx, bool active)
{
    (void)ctx;
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static int spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx;
    HAL_StatusTypeDef st;
    if (tx != NULL && rx != NULL) {
        st = HAL_SPI_TransmitReceive(&s_spi, (uint8_t *)tx, rx, (uint16_t)len, 100U);
    } else if (tx != NULL) {
        st = HAL_SPI_Transmit(&s_spi, (uint8_t *)tx, (uint16_t)len, 100U);
    } else {
        st = HAL_SPI_Receive(&s_spi, rx, (uint16_t)len, 100U);
    }
    return (st == HAL_OK) ? ERR_OK : ERR_IO;
}

static int ext_flash_init(void)
{
    GPIO_InitTypeDef g = { 0 };
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    /* both chip selects high first: the MCP2515 shares the bus */
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAN_CS_PORT, CAN_CS_PIN, GPIO_PIN_SET);
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = FLASH_CS_PIN; HAL_GPIO_Init(FLASH_CS_PORT, &g);
    g.Pin = CAN_CS_PIN;   HAL_GPIO_Init(CAN_CS_PORT, &g);
    g.Mode      = GPIO_MODE_AF_PP;
    g.Alternate = FLASH_SPI_AF;
    g.Pin = FLASH_SCK_PIN;  HAL_GPIO_Init(FLASH_SCK_PORT, &g);
    g.Pin = FLASH_MOSI_PIN; HAL_GPIO_Init(FLASH_MOSI_PORT, &g);
    g.Pull = GPIO_PULLUP;
    g.Pin = FLASH_MISO_PIN; HAL_GPIO_Init(FLASH_MISO_PORT, &g);

    s_spi.Instance               = FLASH_SPI;
    s_spi.Init.Mode              = SPI_MODE_MASTER;
    s_spi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_spi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_spi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_spi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_spi.Init.NSS               = SPI_NSS_SOFT;
    s_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;   /* 16 MHz HSI / 2 = 8 MHz */
    s_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_spi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&s_spi) != HAL_OK) {
        return ERR_IO;
    }
    static const spi_bus_t bus = { spi_transfer, spi_select, NULL };
    int rc = w25qxx_init(&s_w25, &bus, HAL_Delay);
    if (rc == ERR_OK) {
        w25qxx_as_flash(&s_w25, &s_flash);
    }
    return rc;
}

/* ------------------------------------------------- internal flash */
static bool app_vectors_ok(void)
{
    const uint32_t *v = (const uint32_t *)APP_FLASH_BASE;
    return fw_vectors_ok(v[0], v[1], APP_FLASH_BASE, APP_FLASH_END, RAM_START, RAM_END);
}

/* Size of the programmed application: last doubleword that is not erased. */
static uint32_t app_used_size(void)
{
    for (uint32_t a = APP_FLASH_END - 8U; a >= APP_FLASH_BASE; a -= 8U) {
        const uint32_t *w = (const uint32_t *)a;
        if (w[0] != 0xFFFFFFFFUL || w[1] != 0xFFFFFFFFUL) {
            return a + 8U - APP_FLASH_BASE;
        }
    }
    return 0U;
}

static int erase_page(uint32_t addr)
{
    FLASH_EraseInitTypeDef e = { 0 };
    uint32_t bad = 0U;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Banks     = FLASH_BANK_1;                     /* the application stays in bank 1 */
    e.Page      = (addr - BOOT_FLASH_BASE) / PAGE;
    e.NbPages   = 1U;
    return (HAL_FLASHEx_Erase(&e, &bad) == HAL_OK) ? ERR_OK : ERR_IO;
}

/* Copy an image from an external slot into the application area, then
 * check the CRC of the internal flash against the header. */
static int install_from(uint32_t slot, fw_header_t *h)
{
    int rc = fw_slot_verify(&s_flash, slot, APP_MAX_SIZE, h);
    if (rc != ERR_OK) {
        return rc;
    }
    HAL_FLASH_Unlock();
    for (uint32_t off = 0U; off < h->size && rc == ERR_OK; off += PAGE) {
        uint32_t n = (h->size - off < PAGE) ? h->size - off : PAGE;
        memset(s_buf, 0xFF, sizeof s_buf);
        rc = s_flash.read(s_flash.ctx, slot + FW_HDR_SPACE + off, s_buf, n);
        if (rc == ERR_OK) {
            rc = erase_page(APP_FLASH_BASE + off);
        }
        for (uint32_t i = 0U; rc == ERR_OK && i < n; i += 8U) {
            uint64_t dw;
            memcpy(&dw, &s_buf[i], 8U);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_FLASH_BASE + off + i, dw) != HAL_OK) {
                rc = ERR_IO;
            }
        }
    }
    /* erase what is left of a longer previous image, so app_used_size() stays right */
    for (uint32_t a = APP_FLASH_BASE + ((h->size + PAGE - 1U) / PAGE) * PAGE;
         rc == ERR_OK && a < APP_FLASH_BASE + app_used_size(); a += PAGE) {
        rc = erase_page(a);
    }
    HAL_FLASH_Lock();
    if (rc == ERR_OK && crc32((const uint8_t *)APP_FLASH_BASE, h->size) != h->crc) {
        rc = ERR_CRC;
    }
    return rc;
}

/* Save the running application into the BACKUP slot. */
static int backup_app(uint32_t version)
{
    uint32_t size = app_used_size();
    if (size == 0U || size > APP_MAX_SIZE) {
        return ERR_EMPTY;
    }
    fw_writer_t w;
    int rc = fw_writer_begin(&w, &s_flash, FW_SLOT_BACKUP, APP_MAX_SIZE);
    for (uint32_t off = 0U; rc == ERR_OK && off < size; off += PAGE) {
        uint32_t n = (size - off < PAGE) ? size - off : PAGE;
        rc = fw_writer_put(&w, (const void *)(APP_FLASH_BASE + off), n);
    }
    if (rc == ERR_OK) {
        rc = fw_writer_finish(&w, version, crc32((const uint8_t *)APP_FLASH_BASE, size));
    }
    return rc;
}

/* ------------------------------------------------------------ jump */
/* An application that hangs before it starts its own watchdog still gets
 * reset, so an unconfirmed one counts as a failed trial. The application
 * then sets its own timeout (bsp_iwdg_init); the IWDG cannot be stopped. */
static void iwdg_start(void)
{
    IWDG->KR  = 0xCCCCU;                            /* start, turns the LSI on */
    IWDG->KR  = 0x5555U;                            /* unlock PR and RLR */
    IWDG->PR  = IWDG_PRESCALER_256;                 /* 32 kHz / 256: 8 ms per count */
    IWDG->RLR = BOOT_IWDG_MS / 8U;
    uint32_t t0 = HAL_GetTick();
    while (IWDG->SR != 0U && (HAL_GetTick() - t0) < 100U) { }
    IWDG->KR  = 0xAAAAU;                            /* reload */
}

static void jump_to_app(void)
{
    const uint32_t *v = (const uint32_t *)APP_FLASH_BASE;
    iwdg_start();
    HAL_UART_DeInit(&s_uart);
    HAL_SPI_DeInit(&s_spi);
    SysTick->CTRL = 0U;
    SysTick->VAL  = 0U;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;
    SCB->VTOR     = APP_FLASH_BASE;
    __set_MSP(v[0]);
    ((void (*)(void))v[1])();
    for (;;) { }
}

static const char *ver(uint32_t v, char *b)
{
    snprintf(b, 16, "%lu.%lu.%lu", (unsigned long)(v >> 16), (unsigned long)((v >> 8) & 0xFFU),
             (unsigned long)(v & 0xFFU));
    return b;
}

int main(void)
{
    HAL_Init();                                     /* HSI16, SysTick 1 ms */
    uart_init();
    say("[BOOT] bootloader " BOOT_VERSION);

    char a[16], b[16];
    if (ext_flash_init() != ERR_OK) {
        say("[BOOT] no W25Q64, start application");
        if (app_vectors_ok()) {
            jump_to_app();
        }
        say("[BOOT] no valid application, flash one with ST-LINK");
        for (;;) { }
    }

    config_store_t cs;
    fw_boot_t st = { FW_STATE_IDLE, 0U, 0U, 0U };
    config_store_init(&cs, &s_flash, FW_STATE_A_ADDR, FW_STATE_B_ADDR);
    (void)config_store_load(&cs, &st, sizeof st);   /* not found: IDLE, unknown version */
    const fw_boot_t before = st;

    fw_header_t hn, hb;
    bool new_ok    = fw_slot_verify(&s_flash, FW_SLOT_NEW, APP_MAX_SIZE, &hn) == ERR_OK;
    bool backup_ok = fw_slot_verify(&s_flash, FW_SLOT_BACKUP, APP_MAX_SIZE, &hb) == ERR_OK;
    bool app_ok    = app_vectors_ok();

    fw_action_t act = fw_boot_decide(&st, new_ok, backup_ok, app_ok, FW_MAX_TRIES);
    int rc = ERR_OK;
    if (act == FW_ACT_INSTALL) {
        say("[BOOT] install %s (running %s)", ver(hn.version, a), ver(st.prev_version, b));
        /* tries != 0 while PENDING: the backup was taken and a reset hit the
         * programming, so the internal flash is half written; keep the backup */
        bool backed_up = before.state == FW_STATE_PENDING && before.tries != 0U;
        if (backed_up) {
            say("[BOOT] install was interrupted, backup kept");
            if (!backup_ok) {
                st.prev_version = 0U;
            }
        } else if (app_ok) {
            rc = backup_app(st.prev_version);
            say("[BOOT] backup to W25Q64: %s", rc == ERR_OK ? "ok" : "failed");
            if (rc != ERR_OK) {
                st.prev_version = 0U;               /* no rollback possible for this one */
            } else {
                fw_boot_t mark = before;            /* remember it before touching the flash */
                mark.tries = 1U;
                (void)config_store_save(&cs, &mark, sizeof mark);
            }
        }
        uint32_t t0 = HAL_GetTick();
        rc = install_from(FW_SLOT_NEW, &hn);
        say("[BOOT] programmed %lu bytes in %lu ms: %s", (unsigned long)hn.size,
            (unsigned long)(HAL_GetTick() - t0), rc == ERR_OK ? "CRC ok" : "FAILED");
        st.version = hn.version;
    } else if (act == FW_ACT_ROLLBACK) {
        say("[BOOT] rollback to %s", ver(hb.version, a));
        rc = install_from(FW_SLOT_BACKUP, &hb);
        say("[BOOT] rollback: %s", rc == ERR_OK ? "CRC ok" : "FAILED");
    } else if (st.state == FW_STATE_TRIAL) {
        say("[BOOT] trial %lu/%u of %s, waiting for confirm", (unsigned long)st.tries, FW_MAX_TRIES,
            ver(st.version, a));
    }

    if (rc == ERR_OK && memcmp(&st, &before, sizeof st) != 0) {
        rc = config_store_save(&cs, &st, sizeof st);
    }
    if (rc != ERR_OK) {
        say("[BOOT] error %d, retry at next reset", rc);
        if (act != FW_ACT_RUN) {                    /* half-programmed flash: never run it */
            HAL_Delay(1000U);
            NVIC_SystemReset();
        }
    }
    if (app_vectors_ok()) {
        say("[BOOT] start application %s", ver(st.version, a));
        jump_to_app();
    }
    say("[BOOT] no valid application, flash one with ST-LINK");
    for (;;) { }
}
