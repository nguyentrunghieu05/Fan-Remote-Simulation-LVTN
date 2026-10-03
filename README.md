# Bộ điều khiển quạt công nghiệp giám sát từ xa – firmware STM32G0B1RE

Firmware viết mới hoàn toàn cho luận văn tốt nghiệp. Chạy trên board
**NUCLEO-G0B1RE** (STM32G0B1RET6, Cortex-M0+ 64 MHz) với **FreeRTOS**,
kiến trúc phân lớp, và dùng đủ ba chuẩn giao tiếp **UART, SPI, I2C**.

| Chức năng | Chuẩn | Thiết bị |
|---|---|---|
| Điều tốc quạt tự động theo nhiệt độ (PID) | I2C | Cảm biến SHT31 (nhiệt độ, độ ẩm) |
| Màn hình trạng thái tại chỗ | I2C (chung bus) | OLED SSD1306 128x64 |
| Lưu dữ liệu đo khi mất mạng, lưu cấu hình | SPI | Flash W25Q64 (8 MB) |
| Giao tiếp PLC / HMI / SCADA | UART + RS485 | Modbus RTU slave |
| Gửi dữ liệu từ xa, nhận lệnh | UART | Module LoRa Ebyte E220 / E32 |
| Dòng lệnh cấu hình, debug | UART | Cổng COM ảo của ST-LINK |
| Ngõ ra quạt | PWM 25 kHz + DAC 0-10 V | Quạt 4 dây / biến tần / motor EC |
| Đo tốc độ quạt | Timer input capture | Dây tach của quạt |
| An toàn | IWDG + watchdog từng task | Relay báo lỗi |

## 1. Kiến trúc phân lớp

```
 ┌──────────────────────────────────────────────────────────────┐
 │ app/        main.c, app_fsm (máy trạng thái), fan_logic (PID) │
 ├──────────────────────────────────────────────────────────────┤
 │ services/   control, sensors, storage, modbus_server,         │  ← task FreeRTOS
 │             radio_link, display, console, supervisor          │
 ├──────────────────────────────────────────────────────────────┤
 │ middleware/ modbus_slave, pid, ramp, fault, datalog,          │  ← thuần C,
 │             config_store, telemetry, cli, crc, ring_buffer    │    test được trên PC
 ├──────────────────────────────────────────────────────────────┤
 │ drivers/    w25qxx (SPI), sht3x (I2C), ssd1306 (I2C)          │  ← chỉ dùng bus.h
 ├──────────────────────────────────────────────────────────────┤
 │ bsp/        clock, GPIO, UART, SPI, I2C, ADC, PWM, DAC, tach, │  ← lớp DUY NHẤT gọi
 │             RTC, IWDG                                         │    STM32 HAL
 ├──────────────────────────────────────────────────────────────┤
 │ third_party/ STM32G0 HAL, CMSIS, FreeRTOS V11 (mã nguồn mở)   │
 └──────────────────────────────────────────────────────────────┘
```

Nguyên tắc: mỗi lớp chỉ gọi lớp ngay dưới. Driver thiết bị (W25Q, SHT3x,
SSD1306) chỉ biết `spi_bus_t` / `i2c_bus_t` trong `src/common/bus.h`, không
biết HAL. Vì vậy cùng một mã driver chạy trên STM32 và chạy trong unit test
trên máy tính với thiết bị giả lập. Đổi board chỉ cần sửa `config/board.h`
và `src/bsp/`.

### Các task FreeRTOS

| Task | Ưu tiên | Chu kỳ / kích hoạt | Việc |
|---|---|---|---|
| control | 5 | 100 ms | máy trạng thái, phát hiện lỗi, PID, ngõ ra, relay, LED |
| modbus | 4 | ngắt "hết khung" RS485 | Modbus RTU slave |
| sensors | 3 | 10 ms | nút nhấn (chống dội), biến trở, tach, SHT31 mỗi 1 s |
| link | 2 | ngắt UART LoRa / 100 ms | gửi dữ liệu, chờ ACK, nhận lệnh |
| storage | 2 | 1 s | ghi log lên SPI flash theo chu kỳ cấu hình |
| display | 1 | 500 ms | màn hình OLED |
| console | 1 | ngắt UART | dòng lệnh |
| superv | 1 | 250 ms | watchdog từng task, nạp IWDG |
| init | 6 | 1 lần khi khởi động | khởi tạo thiết bị rồi tự xoá |

Giao tiếp giữa các task: một vùng dữ liệu chung có mutex (`sys_data`), một
hàng đợi sự kiện vào task control (đổi chế độ chỉ đi qua đây), và task
notification từ ngắt UART. Mỗi bus (SPI, I2C) có mutex riêng.

### Máy trạng thái

```
 OFF ──nút/lệnh──> MANUAL ──nút/lệnh──> AUTO ──nút/lệnh──> OFF
   \_______________ lỗi nghiêm trọng ________________/
                            │
                         FAULT  (quạt chạy fault_duty = 100 %, bật relay)
                            │  hết lỗi
                            └──> trở về chế độ trước đó
```

Lỗi nghiêm trọng: kẹt quạt (có PWM nhưng tach < ngưỡng, tự giữ đến khi
xoá), mất cảm biến khi đang AUTO, quá nhiệt (có trễ 2 °C). Mất SPI flash
chỉ là cảnh báo.

## 2. Phần cứng và đấu dây (NUCLEO-G0B1RE)

| Chức năng | Chân MCU | Nucleo | Nối tới |
|---|---|---|---|
| Console | PA2 / PA3 (USART2) | ST-LINK USB | máy tính, 115200 8N1 |
| RS485 TX / RX / DE | PA9 / PA10 / PA12 (USART1) | D8 / D2 / CN10-12 | MAX485: DI / RO / DE+RE |
| LoRa TX / RX | PC10 / PC11 (USART3) | CN7-1 / CN7-2 | E220: RXD / TXD |
| LoRa M0 / M1 | PC2 / PC3 | CN7-35 / CN7-37 | E220: M0 / M1 |
| SPI flash SCK / MISO / MOSI / CS | PB13 / PB14 / PB15 / PB12 (SPI2) | CN10 | W25Q64: CLK / DO / DI / CS |
| I2C SCL / SDA | PB8 / PB9 (I2C1) | D15 / D14 | SHT31 + SSD1306, pull-up 4.7 kΩ |
| PWM quạt | PA8 (TIM1_CH1) | D7 | dây PWM quạt 4 dây (qua transistor nếu cần 5 V) |
| 0-10 V | PA4 (DAC1) | A2 | op-amp khuếch đại x3 |
| Tach | PA0 (TIM2_CH1) | A0 | dây tach (open collector, có pull-up) |
| Biến trở tốc độ | PA1 (ADC1_IN1) | A1 | con chạy biến trở 10 kΩ |
| Relay báo lỗi | PC0 | A5 | module relay |
| LED / nút | PA5 / PC13 | LD4 / B1 | có sẵn trên board |

Trước khi đấu dây, kiểm tra lại từng chân với datasheet STM32G0B1 (bảng
Alternate functions) và tài liệu Nucleo UM2324. Toàn bộ chân nằm trong
`config/board.h`.

Không có SPI flash, cảm biến hay OLED firmware vẫn chạy: nó báo "NOT found"
và tiếp tục (đã kiểm tra trên giả lập, xem mục 5).

## 3. Build và nạp

Cần: `arm-none-eabi-gcc` (GNU Arm Toolchain), `make`, STM32CubeProgrammer.

```bash
make                 # -> build/fan_controller.elf / .hex / .bin
make flash           # nạp qua ST-LINK bằng STM32_Programmer_CLI
make test            # chạy unit test trên máy tính (gcc)
make clean
```

Trên Windows, nếu gcc không nằm trong PATH:
`make GCC_PATH="C:/Program Files/Arm GNU Toolchain/bin"`.

Kết quả build hiện tại: 64 KB flash (12 %), 41 KB RAM (28 %, trong đó 32 KB
là heap FreeRTOS), không có cảnh báo với `-Wall -Wextra`.

## 4. Sử dụng

### Dòng lệnh (PuTTY / Tera Term, 115200)

```
status                     đo đạc, trạng thái, lỗi
mode off|manual|auto       đổi chế độ
duty 40 | duty knob        ngõ ra MANUAL: cố định 40 % hoặc theo biến trở
setpoint 27.5              nhiệt độ mục tiêu chế độ AUTO
pid 8 0.4 0                xem / đặt Kp Ki Kd
cfg | cfg set <key> <v> | cfg save | cfg default
log | log dump 50 | log clear     dữ liệu trên SPI flash, xuất CSV
flash                      JEDEC ID và dung lượng flash
i2c                        quét bus I2C
time | time set <unix>     đồng hồ RTC
fault                      xoá lỗi đã chốt (hoặc nhấn giữ nút B1 2 s)
tasks                      task, stack còn trống, % CPU, heap
stats                      bộ đếm Modbus / LoRa
reboot
```

Nhấn nút B1: OFF → MANUAL → AUTO → OFF. LED: sáng liên tục = AUTO, nháy
1 Hz = MANUAL, nháy nhanh = FAULT, chớp ngắn mỗi 2 s = OFF.

### Bảng thanh ghi Modbus RTU (slave id 1, 9600 8N1)

Input register (FC 04): 0 nhiệt độ (0.01 °C), 1 độ ẩm (0.01 %), 2 rpm,
3 ngõ ra (0.1 %), 4 trạng thái, 5 bit lỗi, 6 biến trở, 7-8 uptime, 9-10 số
bản ghi log, 11 phiên bản.

Holding register (FC 03/06/16): 0 lệnh chế độ, 1 duty tay, 2 setpoint,
3-5 Kp Ki Kd (x100), 6 duty tối thiểu, 7 duty khi lỗi, 8 ngưỡng quá nhiệt,
9 tốc độ ramp, 10 chu kỳ log, 11 chu kỳ gửi, 12 địa chỉ, 13 dùng biến trở,
14 ngưỡng kẹt quạt, 20 hành động (1 lưu, 2 xoá lỗi, 3 khởi động lại).
Chi tiết đơn vị trong `src/services/modbus_server.h`.

Cuối khung Modbus được phát hiện bằng phần cứng: bộ đếm receiver-timeout
của USART đặt 3,5 ký tự, chân DE của MAX485 cũng do USART tự điều khiển.

### Giao thức LoRa (store-and-forward)

```
Controller -> gateway : $FAN,<node>,<seq>,<unix>,<t_c100>,<rh_c100>,<rpm>,<duty_c10>,<state>,<faults>*CS
Gateway -> controller : $ACK,<node>,<seq>*CS
Gateway -> controller : $CMD,<node>,mode|duty|setpoint|time,<value>*CS
```

Mỗi bản ghi trong flash chỉ được đánh dấu "đã gửi" khi nhận đúng ACK. Mất
sóng thì dữ liệu nằm chờ trong flash, có sóng lại thì gửi bù.

### Bố trí SPI flash

| Vùng | Địa chỉ | Nội dung |
|---|---|---|
| Cấu hình A / B | 0x000000 / 0x001000 | 2 bản sao, CRC-32, luân phiên: mất điện giữa chừng vẫn còn bản cũ |
| Log đo đạc | 0x010000 → hết chip | vòng tròn, bản ghi 32 byte có số thứ tự + CRC-16 |

Với W25Q64 và chu kỳ log 10 s: khoảng 260.000 bản ghi, tức khoảng 30 ngày.
Cờ "đã gửi" được ghi bằng cách lập trình byte 0xFF thành 0x00 (NOR flash
cho phép xoá bit 1 → 0 không cần xoá sector).

## 5. Kiểm thử

**Unit test trên máy tính** (`make test`): 23 bài, 391 phép kiểm tra, biên
dịch với AddressSanitizer và UBSan. Gồm: vector chuẩn của CRC, khung Modbus
đúng / sai CRC / sai địa chỉ / broadcast, PID chống bão hoà tích phân, mô
phỏng nhiệt vòng kín hội tụ về setpoint, log flash qua quấn vòng và khởi
động lại, mất điện giữa lúc ghi log và lúc lưu cấu hình, driver W25Q chạy
với chip giả lập giải mã lệnh SPI, driver SHT3x với CRC sai và NACK.

**Chạy firmware thật trên giả lập Renode** (`tests/renode/`): nạp file
`.elf` đã build vào mô hình STM32G0 của Renode 1.15.3, giao tiếp qua
USART2. 3 kịch bản đạt: khởi động khi thiếu phần cứng phụ, chế độ AUTO mất
cảm biến chuyển sang FAULT với ngõ ra 100 %, điều khiển bằng dòng lệnh.

```bash
renode-test tests/renode/fan_controller.robot
```

Giả lập không thay được phần cứng thật: mô hình Renode không có SPI flash,
SHT31, OLED, chưa hỗ trợ receiver-timeout của USART và input capture, và
lệnh `time set` làm Renode dừng (lỗi mô hình RTC). Các phần đó cần thử trên
board, dùng Logic Analyzer để chụp sóng SPI, I2C, UART cho chương thử nghiệm.

## 6. Cấu trúc thư mục

```
config/        board.h (chân), FreeRTOSConfig.h, stm32g0xx_hal_conf.h
linker/        STM32G0B1RETX_FLASH.ld (512 KB flash, 144 KB RAM)
startup/       startup_stm32g0b1xx.s, system_stm32g0xx.c (của ST)
src/common/    err.h, bus.h
src/bsp/       lớp phần cứng
src/drivers/   W25Qxx, SHT3x, SSD1306
src/middleware/ thư viện thuần C
src/services/  các task
src/app/       main, máy trạng thái, luật điều khiển
tests/         unit test (PC) + tests/renode (giả lập)
third_party/   HAL STM32G0 (BSD-3), CMSIS (Apache-2.0), FreeRTOS (MIT)
```
