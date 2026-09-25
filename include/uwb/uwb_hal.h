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

// Bật/tắt chế độ tiết kiệm năng lượng (duty-cycle thưa) sau khi đã unlock.
// Vẫn tiếp tục ranging (cần để phát hiện lúc rời xa quá R_LOCK -> relock).
//
// CHỈ đo thưa (mỗi round cách nhau nhiều giây) khi khoảng cách gần nhất đo
// được còn < nearThresholdM (mặc định 1.0 m - đúng vùng unlock, lúc này
// người dùng gần như chắc chắn đứng ngay cạnh xe, không cần theo dõi sát).
// Ngay khi một mẫu cho thấy khoảng cách >= nearThresholdM (đang rời xa,
// đúng lúc cần quyết định relock chính xác) - TỰ ĐỘNG quay lại đo full-rate
// với bộ lọc median+EMA như lúc tracking bình thường, không đợi lệnh nào
// khác. (Trước đây đo thưa suốt cả giai đoạn sau unlock, kể cả khi đã rời
// xa 1-2m để chờ relock - mỗi mẫu cách nhau ~2.5s không được lọc gì, dễ
// nhảy vọt sai và làm relock chậm/nhảy cảm.)
// Chỉ implement bên phía car (RX/responder) - phía keyfob (TX) không cần
// và không gọi hàm này.
void UWB_SetLowPowerMode(bool enabled, float nearThresholdM = 1.0f);
