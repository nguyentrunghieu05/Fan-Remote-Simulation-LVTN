/*
 * Application behind the bootloader: the first 32 KB of flash belong to
 * the bootloader (boot.ld). SystemInit points SCB->VTOR at 0x08008000
 * (USER_VECT_TAB_ADDRESS, VECT_TAB_OFFSET set by make BOOT=1).
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 480K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 144K
}

INCLUDE sections.ld
