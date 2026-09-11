#pragma once

#include <Arduino.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Firebase Manager (Gateway only)
// -----------------------------------------------------------------------------
// Chỉ đọc/ghi theo path do nơi gọi truyền vào - KHÔNG biết gì về schema
// (SecureKeys, Vehicles...). Schema do bên phụ trách data định nghĩa,
// module này chỉ là transport tới Realtime Database.

// Khởi tạo kết nối Firebase (dùng Legacy Database Secret - tạm thời,
// xem ghi chú bảo mật trong cloud_config.h). Idempotent.
// PHẢI gọi WiFi_Connect() thành công trước khi gọi hàm này.
bool Firebase_Init();

bool Firebase_IsReady();

// Đọc 1 giá trị string tại path. Trả false nếu path không tồn tại hoặc
// lỗi kết nối - outValue không được sửa nếu trả false.
bool Firebase_ReadString(const char* path, String& outValue);

// Ghi 1 giá trị string tại path (tạo mới hoặc ghi đè).
bool Firebase_WriteString(const char* path, const String& value);

// Ghi 1 giá trị bool tại path (tiện cho các field như ble_connected).
bool Firebase_WriteBool(const char* path, bool value);