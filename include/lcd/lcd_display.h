#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "can/payloads.h"

// -----------------------------------------------------------------------------
// LCD Display (car hoặc gateway - build được cho cả 2, xem lcd_config.h)
// -----------------------------------------------------------------------------
// Hiển thị trạng thái hệ thống lên LCD 1602 I2C:
//
//   Màn hình chính:          Gateway, car offline:
//   +----------------+       +----------------+
//   |BLE AUTH OK LOCK|       |CAR OFFLINE LOCK|
//   |UWB  0.75m HOLD |       |WiFi:OK FB:OK   |
//   +----------------+       +----------------+
//
// Mô hình: các hàm LCD_Set*() chỉ cập nhật 1 struct trạng thái (critical
// section ngắn, gọi được từ bất kỳ task nào); 1 task riêng khởi tạo I2C,
// render định kỳ và là nơi DUY NHẤT chạm vào bus I2C. Task tự dò LCD, tự
// init lại nếu LCD bị rút ra cắm lại.
//
// Không có LCD thì các hàm Set*() vẫn gọi an toàn (chỉ cập nhật state).

enum LcdLinkState : uint8_t
{
    LCD_LINK_PENDING = 0, // đang kết nối  -> ".."
    LCD_LINK_OK,          //               -> "OK"
    LCD_LINK_FAIL,        //               -> "NG"
};

// Tạo task LCD (task tự Wire.begin + dò địa chỉ + init, không block).
// Trả false chỉ khi không tạo được task. Gọi 1 lần trong setup().
bool LCD_Init();

// Chỉ gateway hiển thị (car không dùng mạng).
void LCD_SetWifiState(LcdLinkState state);
void LCD_SetFirebaseState(LcdLinkState state);

// Gateway: gọi khi nhận CAN_ID_CAR_STATUS hợp lệ (làm mới mốc online).
// Car: gọi định kỳ với trạng thái tại chỗ.
void LCD_SetCarStatus(const CarStatusPayload& status);

// Gateway: trạng thái actuator đã thi hành. Car: lệnh lock/unlock đã gửi thành công.
void LCD_SetLockState(bool unlocked);

// Hiện thông báo tạm thời (đè màn hình chính) trong durationMs.
// Chuỗi dài hơn LCD_COLS bị cắt.
void LCD_ShowMessage(const char* line0, const char* line1, uint32_t durationMs);
