# Host unit tests: compile the portable layers with the PC compiler.
#   make -C tests        -> build and run
CC      ?= gcc
SRC_DIR  = ../src
CFLAGS   = -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined \
           -I$(SRC_DIR)/common -I$(SRC_DIR)/middleware -I$(SRC_DIR)/drivers -I$(SRC_DIR)/app -I.
LDLIBS   = -lm

SRCS = test_main.c test_basic.c test_storage.c test_drivers.c test_app.c test_i2c_devices.c test_mcp2515.c fake_flash.c \
       $(SRC_DIR)/middleware/ring_buffer.c $(SRC_DIR)/middleware/crc.c \
       $(SRC_DIR)/middleware/modbus_slave.c $(SRC_DIR)/middleware/pid.c \
       $(SRC_DIR)/middleware/ramp.c $(SRC_DIR)/middleware/fault.c \
       $(SRC_DIR)/middleware/timeconv.c $(SRC_DIR)/middleware/cli.c \
       $(SRC_DIR)/middleware/datalog.c $(SRC_DIR)/middleware/config_store.c \
       $(SRC_DIR)/middleware/telemetry.c \
       $(SRC_DIR)/drivers/w25qxx.c $(SRC_DIR)/drivers/sht3x.c $(SRC_DIR)/drivers/ssd1306.c \
       $(SRC_DIR)/drivers/bme280.c $(SRC_DIR)/drivers/lcd1602.c \
       $(SRC_DIR)/drivers/mcp2515.c \
       $(SRC_DIR)/app/app_fsm.c $(SRC_DIR)/app/fan_logic.c

all: run

unit_tests: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $@ $(LDLIBS)

run: unit_tests
	./unit_tests

clean:
	rm -f unit_tests

.PHONY: all run clean
