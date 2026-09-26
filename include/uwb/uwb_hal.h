#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // size_t

// -----------------------------------------------------------------------------
// UWB HAL (Hardware Abstraction Layer)
// -----------------------------------------------------------------------------
// Interface chung cho cả mock (chưa có phần cứng) và driver thật (DW3000EVB).
// Code gọi (main.cpp) chỉ biết tới các hàm này - không quan tâm implementation
// nào đang được compile vào.
//
// Chọn implementation qua build flag UWB_USE_MOCK (xem platformio.ini):
//   - UWB_USE_MOCK=1 -> build src/uwb/uwb_mock.cpp
//   - không định nghĩa  -> build src/uwb/uwb_dw3000_rx.cpp hoặc uwb_dw3000_tx.cpp
//
// Thiết kế task-based: đo khoảng cách là việc có độ trễ (radio exchange),
// không nên gọi đồng bộ chặn task khác. Ranging chạy nền trong 1 task
// riêng (do implementation tự quản lý), kết quả đọc ra qua
// UWB_GetLastDistance() - non-blocking, luôn trả ngay giá trị gần nhất.

// Khởi tạo phần cứng/mock. Idempotent - gọi nhiều lần vẫn an toàn.
bool UWB_Init();

// Nạp K_session (16 byte AES-128) vào DW3000 trước khi bắt đầu ranging.
// Gọi sau khi BLE AUTH_OK - khi nonce đã có và HKDF đã tính xong.
// Phải gọi TRƯỚC UWB_StartRanging() để STS được bind theo phiên.
// Phía mock: ghi log rồi trả true (no-op về phần cứng).
bool UWB_SetSessionKey(const uint8_t* key, size_t len);

// Bắt đầu vòng lặp ranging liên tục (chạy nền). Gọi sau UWB_SetSessionKey().
bool UWB_StartRanging();

// Dừng ranging (khi disconnect / relock / hết phiên xác thực).
bool UWB_StopRanging();

// Đọc kết quả đo gần nhất, non-blocking.
// Trả false nếu chưa có phép đo nào thành công (ranging chưa chạy,
// hoặc mới start chưa đủ thời gian có kết quả đầu tiên).
bool UWB_GetLastDistance(float& outMeters);

// Như UWB_GetLastDistance() nhưng kèm số thứ tự mẫu (tăng 1 sau mỗi mẫu đã lọc
// mới được công bố), để phía gọi đếm "N mẫu liên tiếp" mà không đếm trùng một
// mẫu bị đọc lại nhiều lần. Đọc khoảng cách và seq nhất quán với nhau.
// Chỉ implement bên phía car (RX/responder) - phía keyfob (TX) không gọi hàm này.
bool UWB_GetLastSample(float& outMeters, uint32_t& outSeq);
