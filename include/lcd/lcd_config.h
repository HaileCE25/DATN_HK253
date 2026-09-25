#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// LCD Subsystem Configuration (LCD 1602 + mạch I2C PCF8574)
// -----------------------------------------------------------------------------
// GPIO11/12 = header SDA/SCL + cổng Grove trên YoloUNO. Chỉ car dùng LCD.

#if !defined(IS_CAR)
#error "lcd_config.h chi danh cho car (IS_CAR)"
#endif

// Car đang dùng: CAN 17/18, DW3000 SPI 21/38/6/7 + RST 5 + DEBUG 4,
// RC522 47/48/10/9/8, USB 19/20.
constexpr int LCD_SDA_PIN = 11;
constexpr int LCD_SCL_PIN = 12;

// Core 0: tách khỏi core 1 nơi RangingTask UWB (priority 2) busy-wait trong cửa
// sổ DS-TWR. Ngắt I2C gắn vào core gọi Wire.begin (chính task LCD) -> cũng ở
// core 0. Priority 1 < TaskBLE (2) nên không làm trễ BLE.
constexpr int LCD_TASK_CORE     = 0;
constexpr int LCD_TASK_PRIORITY = 1;

constexpr uint32_t LCD_I2C_FREQ_HZ = 100000;

// PCF8574 = 0x27, PCF8574A = 0x3F - dò lần lượt.
constexpr uint8_t LCD_I2C_ADDR_PRIMARY   = 0x27;
constexpr uint8_t LCD_I2C_ADDR_SECONDARY = 0x3F;

constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_ROWS = 2;

constexpr uint32_t LCD_REFRESH_MS = 200;

// Chu kỳ dò LCD trên bus I2C: chưa thấy/mất ACK -> ngừng vẽ, thử lại; có ACK
// -> init lại HD44780 (LCD bị rút nguồn/dây khi đang chạy sẽ mất cấu hình).
constexpr uint32_t LCD_PROBE_MS = 1000;
