##############################################################################
# Fan controller firmware - STM32G0B1RE (NUCLEO-G0B1RE)
#
#   make            build build/fan_controller.{elf,hex,bin}
#   make BOOT=1     same, linked at 0x08008000 to run behind the bootloader
#   make flash      program the board (STM32CubeProgrammer CLI over ST-LINK)
#   make test       build and run the unit tests on the PC
#   make clean
#
# Toolchain: GNU Arm Embedded (arm-none-eabi-gcc). Set GCC_PATH if it is
# not in PATH, e.g.  make GCC_PATH="C:/arm-gnu-toolchain/bin"
##############################################################################

TARGET   = fan_controller
BUILD    = build

ifdef GCC_PATH
PREFIX = $(GCC_PATH)/arm-none-eabi-
else
PREFIX = arm-none-eabi-
endif
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

# ---------------------------------------------------------------- sources
HAL_DIR  = third_party/STM32G0xx_HAL_Driver
RTOS_DIR = third_party/FreeRTOS

APP_SRC = $(wildcard src/app/*.c) \
          $(wildcard src/services/*.c) \
          $(wildcard src/middleware/*.c) \
          $(wildcard src/drivers/*.c) \
          $(wildcard src/bsp/*.c)

HAL_SRC = $(filter-out %_template.c,$(wildcard $(HAL_DIR)/Src/stm32g0xx_hal*.c))

RTOS_SRC = $(RTOS_DIR)/tasks.c $(RTOS_DIR)/queue.c $(RTOS_DIR)/list.c \
           $(RTOS_DIR)/timers.c $(RTOS_DIR)/event_groups.c $(RTOS_DIR)/stream_buffer.c \
           $(RTOS_DIR)/portable/GCC/ARM_CM0/port.c \
           $(RTOS_DIR)/portable/GCC/ARM_CM0/portasm.c \
           $(RTOS_DIR)/portable/MemMang/heap_4.c

C_SRC   = $(APP_SRC) $(HAL_SRC) $(RTOS_SRC) startup/system_stm32g0xx.c
ASM_SRC = startup/startup_stm32g0b1xx.s

INCLUDES = -Iconfig -Isrc/common -Isrc/bsp -Isrc/drivers -Isrc/middleware \
           -Isrc/services -Isrc/app \
           -I$(HAL_DIR)/Inc -Ithird_party/CMSIS/Device/Include -Ithird_party/CMSIS/Include \
           -I$(RTOS_DIR)/include -I$(RTOS_DIR)/portable/GCC/ARM_CM0

# ---------------------------------------------------------------- flags
MCU     = -mcpu=cortex-m0plus -mthumb
DEFS    = -DSTM32G0B1xx -DUSE_HAL_DRIVER
OPT     = -Os -g3
CFLAGS  = $(MCU) $(DEFS) $(INCLUDES) $(OPT) -std=gnu11 -Wall -Wextra \
          -ffunction-sections -fdata-sections -fno-common -MMD -MP
# Third-party code is compiled with fewer warnings: it is not ours to fix.
CFLAGS_3P = $(filter-out -Wextra,$(CFLAGS)) -Wno-unused-parameter
ASFLAGS = $(MCU) $(DEFS) -g3

# BOOT=1: the application starts at 0x08008000, behind the bootloader, and
# SystemInit moves the vector table there. Use a separate build directory
# so the two layouts never share object files.
BOOT ?= 0
ifeq ($(BOOT),1)
LDSCRIPT = linker/app.ld
DEFS    += -DUSER_VECT_TAB_ADDRESS -DVECT_TAB_OFFSET=0x00008000U
BUILD    = build_app
else
LDSCRIPT = linker/STM32G0B1RETX_FLASH.ld
endif
LDFLAGS  = $(MCU) -T$(LDSCRIPT) -Llinker --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map,--cref \
           -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

# ---------------------------------------------------------------- rules
OBJS = $(addprefix $(BUILD)/,$(C_SRC:.c=.o)) $(addprefix $(BUILD)/,$(ASM_SRC:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
	@$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS_3P) $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT) linker/sections.ld
	@echo "LD  $@"
	@$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/%.hex: $(BUILD)/%.elf
	@$(OBJCOPY) -O ihex $< $@

$(BUILD)/%.bin: $(BUILD)/%.elf
	@$(OBJCOPY) -O binary -S $< $@

flash: all
	STM32_Programmer_CLI -c port=SWD -w $(BUILD)/$(TARGET).hex -v -rst

# ---------------------------------------------------------------- bootloader
# make boot / make flash-boot: the bootloader in the first 32 KB. It reuses
# the portable code (fw_image, CRC, config_store, W25Q64 driver) and a few
# HAL modules, no RTOS. The application must then be built with BOOT=1.
BOOT_BUILD = build_boot
BOOT_HAL   = $(addprefix $(HAL_DIR)/Src/stm32g0xx_hal,.c _cortex.c _rcc.c _rcc_ex.c _gpio.c _dma.c \
             _spi.c _uart.c _uart_ex.c _flash.c _flash_ex.c _pwr_ex.c)
BOOT_C_SRC = boot/boot.c src/middleware/fw_image.c src/middleware/crc.c \
             src/middleware/config_store.c src/drivers/w25qxx.c \
             startup/system_stm32g0xx.c $(BOOT_HAL)
BOOT_OBJS  = $(addprefix $(BOOT_BUILD)/,$(BOOT_C_SRC:.c=.o)) $(addprefix $(BOOT_BUILD)/,$(ASM_SRC:.s=.o))

boot: $(BOOT_BUILD)/boot.elf $(BOOT_BUILD)/boot.hex $(BOOT_BUILD)/boot.bin
	@$(SIZE) $<

$(BOOT_BUILD)/boot/%.o: boot/%.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BOOT_BUILD)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS) $< -o $@

$(BOOT_BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC  $<"
	@$(CC) -c $(CFLAGS_3P) $< -o $@

$(BOOT_BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS  $<"
	@$(AS) -c $(ASFLAGS) $< -o $@

$(BOOT_BUILD)/boot.elf: $(BOOT_OBJS) linker/boot.ld linker/sections.ld
	@echo "LD  $@"
	@$(CC) $(BOOT_OBJS) $(MCU) -Tlinker/boot.ld -Llinker --specs=nano.specs --specs=nosys.specs \
	   -Wl,--gc-sections -Wl,-Map=$(BOOT_BUILD)/boot.map -Wl,--print-memory-usage \
	   -Wl,--no-warn-rwx-segments -o $@

$(BOOT_BUILD)/boot.hex: $(BOOT_BUILD)/boot.elf
	@$(OBJCOPY) -O ihex $< $@

$(BOOT_BUILD)/boot.bin: $(BOOT_BUILD)/boot.elf
	@$(OBJCOPY) -O binary -S $< $@

flash-boot: boot
	STM32_Programmer_CLI -c port=SWD -w $(BOOT_BUILD)/boot.hex -v -rst

test:
	$(MAKE) -C tests

clean:
	rm -rf build build_app build_boot
	$(MAKE) -C tests clean

-include $(OBJS:.o=.d)

.PHONY: all flash test clean boot flash-boot
