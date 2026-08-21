#pragma once

#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// UWB HAL (Hardware Abstraction Layer)
// -----------------------------------------------------------------------------
// Interface chung cho cả mock (chưa có phần cứng) và driver thật (DW3000EVB,
// dùng Fhilb/DW3000_Arduino cho M1). Code gọi (main.cpp) chỉ biết tới các
// hàm này - không quan tâm implementation nào đang được compile vào.
//
// Chọn implementation qua build flag UWB_USE_MOCK (xem platformio.ini):
//   - UWB_USE_MOCK=1 -> build src/uwb/uwb_mock.cpp
//   - không định nghĩa  -> build src/uwb/uwb_dw3000.cpp (driver thật)
//
// Thiết kế task-based: đo khoảng cách là việc có độ trễ (radio exchange),
// không nên gọi đồng bộ chặn task khác. Ranging chạy nền trong 1 task
// riêng (do implementation tự quản lý), kết quả đọc ra qua
// UWB_GetLastDistance() - non-blocking, luôn trả ngay giá trị gần nhất.

// Khởi tạo phần cứng/mock. Idempotent - gọi nhiều lần vẫn an toàn.
bool UWB_Init();

// Bắt đầu vòng lặp ranging liên tục (chạy nền). Gọi sau khi BLE
// AUTH_OK, tương ứng bước "Bật UWB Ranging" trong TODO hiện có.
bool UWB_StartRanging();

// Dừng ranging (khi disconnect / relock / hết phiên xác thực).
bool UWB_StopRanging();

// Đọc kết quả đo gần nhất, non-blocking.
// Trả false nếu chưa có phép đo nào thành công (ranging chưa chạy,
// hoặc mới start chưa đủ thời gian có kết quả đầu tiên).
bool UWB_GetLastDistance(float& outMeters);