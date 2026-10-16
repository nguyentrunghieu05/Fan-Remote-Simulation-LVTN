/*
 * Bootloader: the first 32 KB of flash (boot/boot.c, make boot).
 * The application follows at 0x08008000 (app.ld).
 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 144K
}

INCLUDE sections.ld
