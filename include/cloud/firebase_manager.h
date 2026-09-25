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
// Chỉ thành công khi WiFi đã kết nối (WiFi_IsConnected()); trả false nếu chưa.
bool Firebase_Init();

bool Firebase_IsReady();

// Đọc 1 giá trị string tại path. Trả false nếu path không tồn tại hoặc
// lỗi kết nối - outValue không được sửa nếu trả false. Chỉ tự log lỗi kết
// nối/quyền; path không tồn tại thì im lặng, nơi gọi tự log nếu cần.
bool Firebase_ReadString(const char* path, String& outValue);

// Lọc các node con của path có field `child` == value (orderBy/equalTo của
// RTDB), trả JSON thô dạng {"<push_id>": {...}, ...}. Không có node nào khớp
// (kể cả path chưa tồn tại) -> true với outJson = "{}". Trả false CHỈ khi lỗi
// kết nối/quyền - nơi gọi phân biệt được "không có dữ liệu" với "không tra được".
// Rules phải có ".indexOn": ["<child>"] tại path, không thì RTDB từ chối query.
bool Firebase_QueryEqualTo(const char* path, const char* child, const char* value, String& outJson);

// Ghi 1 giá trị string tại path (tạo mới hoặc ghi đè).
bool Firebase_WriteString(const char* path, const String& value);

// Ghi 1 giá trị bool tại path (tiện cho các field như ble_connected).
bool Firebase_WriteBool(const char* path, bool value);