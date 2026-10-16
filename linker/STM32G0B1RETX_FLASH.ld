/*
 * Linker script for STM32G0B1RET6: 512 KB flash, 144 KB SRAM.
 * Application alone at 0x08000000 (default build, no bootloader).
 * With the bootloader use app.ld (make BOOT=1).
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 144K
}

INCLUDE sections.ld
