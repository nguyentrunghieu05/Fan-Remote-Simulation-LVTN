# Bộ điều khiển quạt công nghiệp giám sát từ xa – firmware STM32G0B1RE

Firmware viết mới hoàn toàn cho luận văn tốt nghiệp. Chạy trên board
**NUCLEO-G0B1RE** (STM32G0B1RET6, Cortex-M0+ 64 MHz) với **FreeRTOS**,
kiến trúc phân lớp, và dùng đủ ba chuẩn giao tiếp **UART, SPI, I2C**.

| Chức năng | Chuẩn | Thiết bị |
|---|---|---|
| Điều tốc quạt tự động theo nhiệt độ (PID) | I2C | Cảm biến BME280 / BMP280 hoặc SHT31 (tự nhận) |
| Màn hình trạng thái tại chỗ | I2C (chung bus) | LCD 1602 + PCF8574 hoặc OLED SSD1306 (tự nhận) |
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
BME280, SSD1306, LCD1602) chỉ biết `spi_bus_t` / `i2c_bus_t` trong `src/common/bus.h`, không
biết HAL. Vì vậy cùng một mã driver chạy trên STM32 và chạy trong unit test
trên máy tính với thiết bị giả lập. Đổi board chỉ cần sửa `config/board.h`
và `src/bsp/`.

### Các task FreeRTOS

| Task | Ưu tiên | Chu kỳ / kích hoạt | Việc |
|---|---|---|---|
| control | 5 | 100 ms | máy trạng thái, phát hiện lỗi, PID, ngõ ra, relay, LED |
| modbus | 4 | ngắt "hết khung" RS485 | Modbus RTU slave |
| sensors | 3 | 10 ms | nút nhấn (chống dội), biến trở, tach, cảm biến nhiệt mỗi 1 s |
| link | 2 | ngắt UART LoRa / 100 ms | gửi dữ liệu, chờ ACK, nhận lệnh |
| storage | 2 | 1 s | ghi log lên SPI flash theo chu kỳ cấu hình |
| display | 1 | 500 ms | màn hình LCD 1602 hoặc OLED |
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
| I2C SCL / SDA | PB8 / PB9 (I2C1) | D15 / D14 | BME280 + LCD 1602 (hoặc SHT31 + SSD1306), pull-up 4.7 kΩ |
| PWM quạt | PA8 (TIM1_CH1) | D7 | dây PWM quạt 4 dây (qua transistor nếu cần 5 V) |
| 0-10 V | PA4 (DAC1) | A2 | op-amp khuếch đại x3 |
| Tach | PA0 (TIM2_CH1) | A0 | dây tach (open collector, có pull-up) |
| Biến trở tốc độ | PA1 (ADC1_IN1) | A1 | con chạy biến trở 10 kΩ |
| Relay báo lỗi | PC0 | A5 | module relay |
| LED / nút | PA5 / PC13 | LD4 / B1 | có sẵn trên board |

Trước khi đấu dây, kiểm tra lại từng chân với datasheet STM32G0B1 (bảng
Alternate functions) và tài liệu Nucleo UM2324. Toàn bộ chân nằm trong
`config/board.h`.

### Cảm biến và màn hình I2C: tự nhận

Lúc khởi động (và thử lại mỗi 10 s nếu chưa thấy) firmware dò bus I2C:

| Loại | Thử lần lượt | Địa chỉ |
|---|---|---|
| Cảm biến nhiệt | SHT3x, rồi BME280 / BMP280 (đọc thanh ghi ID 0xD0: 0x60 / 0x58) | 0x44, 0x45, rồi 0x76, 0x77 |
| Màn hình | OLED SSD1306, rồi LCD 16x2 qua PCF8574 | 0x3C, rồi 0x27, 0x3F |

Log khởi động cho biết đã nhận gì, ví dụ `SENS BME280 found at 0x76 (id 0x60)`
và `DISP LCD 16x2 (PCF8574) found at 0x27`. Lệnh `i2c` liệt kê địa chỉ đang trả
lời kèm tên đoán. BMP280 không có độ ẩm nên lệnh `status` báo độ ẩm 0.

Đấu dây: VCC 3.3 V, GND, SDA vào D14, SCL vào D15. Phần lớn module BME280 và
PCF8574 đã có sẵn điện trở kéo lên. LCD 1602 thường cần 5 V mới đủ tương phản;
nếu cấp 5 V cho module PCF8574 thì chân SDA/SCL bị kéo lên 5 V: PB8/PB9 chịu
được 5 V (FT) nhưng hãy kiểm tra lại datasheet, hoặc dùng module chuyển mức.
Chỉnh biến trở xanh sau LCD nếu thấy màn sáng mà không có chữ.

Không có SPI flash, cảm biến hay màn hình firmware vẫn chạy: nó báo "NOT found"
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

### Mở bằng STM32CubeIDE

Thư mục này đã là một project STM32CubeIDE (`.project`, `.cproject`), dùng
chính `Makefile` ở trên để build, nên IDE và dòng lệnh luôn ra cùng một file.

1. Giải nén vào `D:\Fan-LVTN\fan-controller-fw`.
2. CubeIDE: **File > Import > General > Existing Projects into Workspace**,
   chọn thư mục trên, bấm Finish (không tick "Copy projects into workspace").
3. Bấm Build (búa). CubeIDE tự đưa trình biên dịch và `make` có sẵn của nó vào
   PATH. Kết quả: `build/fan_controller.elf`.
4. Debug: chạy cấu hình `fan_controller Debug` (ST-LINK, có hiển thị task
   FreeRTOS). Nếu IDE báo lỗi cấu hình, chuột phải `build/fan_controller.elf`
   > **Debug As > STM32 C/C++ Application** để IDE tự tạo lại.

`tools/cubemx/fan_controller_cubemx.ioc` mô tả cùng sơ đồ chân, xung clock
64 MHz và thông số ngoại vi như `config/board.h`, để mở bằng STM32CubeMX xem
pinout, clock tree và đưa hình vào luận văn. Firmware không sinh code từ
CubeMX (lớp BSP viết tay), nên **đừng bấm Generate Code vào thư mục
firmware**. Nếu muốn thử, CubeMX sẽ tạo project riêng `fan_controller_cubemx`
trong `tools/cubemx/`. Các file `.ioc` / `.cproject` này được viết tay, chưa
mở thử trên CubeIDE / CubeMX thật; nếu IDE báo lỗi khi import, hãy gửi ảnh lỗi.

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

**Unit test trên máy tính** (`make test`): 444 phép kiểm tra, biên
dịch với AddressSanitizer và UBSan. Gồm: vector chuẩn của CRC, khung Modbus
đúng / sai CRC / sai địa chỉ / broadcast, PID chống bão hoà tích phân, mô
phỏng nhiệt vòng kín hội tụ về setpoint, log flash qua quấn vòng và khởi
động lại, mất điện giữa lúc ghi log và lúc lưu cấu hình, driver W25Q chạy
với chip giả lập giải mã lệnh SPI, driver SHT3x với CRC sai và NACK, driver BME280 với vector
bù trừ trong datasheet, driver LCD1602 với mô hình HD44780 giải mã từng byte PCF8574.

**App giả lập trên trình duyệt** (`tools/simulator/fan_sim.html`): mở trực
tiếp bằng Chrome / Edge, không cần cài gì. Các file C thật của firmware
(`app_fsm`, `fan_logic`, `pid`, `ramp`, `fault`, `modbus_slave`, `crc`,
`telemetry`, `datalog`, `cli`, driver `w25qxx`, `sht3x`, `ssd1306`) được biên
dịch sang JavaScript (`fan_sim_core.js`, bằng `build_sim.sh`: clang → WebAssembly
→ wasm2js). `sim_board.c` thay phần cứng: mô hình nhiệt của máy và quạt, chip
W25Q64 / SHT31 / SSD1306 (nhánh SHT31 + OLED của bộ tự nhận) giả lập ở mức byte trên bus SPI / I2C, và một vòng
100 ms gọi các bước giống các task trên chip. Trong app có màn hình OLED vẽ từ
GDDRAM, LED, relay, đồ thị đáp ứng PID, console CLI, Modbus master (gửi khung
có CRC, giải mã trả lời và exception), LoRa gateway ($FAN / $ACK / $CMD), và
các nút gây lỗi (kẹt quạt, rút cảm biến, mất sóng LoRa, tháo flash).
Chạy nhanh trên PC không cần trình duyệt: xem dòng cuối `build_sim.sh`.

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
src/drivers/   W25Qxx, SHT3x, BME280, SSD1306, LCD1602
src/middleware/ thư viện thuần C
src/services/  các task
src/app/       main, máy trạng thái, luật điều khiển
tests/         unit test (PC) + tests/renode (giả lập)
tools/simulator/ app giả lập trên trình duyệt (fan_sim.html)
tools/cubemx/  file .ioc để xem pinout / clock bằng STM32CubeMX
third_party/   HAL STM32G0 (BSD-3), CMSIS (Apache-2.0), FreeRTOS (MIT)
```
