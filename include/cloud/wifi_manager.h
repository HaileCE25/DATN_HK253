#pragma once

#include <stdbool.h>

// -----------------------------------------------------------------------------
// WiFi Manager (Gateway only)
// -----------------------------------------------------------------------------
// Chỉ quản lý kết nối WiFi. Không biết gì về Firebase hay dữ liệu ứng dụng
// - đúng nguyên tắc tách lớp đã áp dụng cho các module khác (can/, uwb/).

// Kết nối WiFi, blocking tối đa WIFI_CONNECT_TIMEOUT_MS (xem cloud_config.h).
// Trả false nếu hết timeout mà chưa connect được.
bool WiFi_Connect();

bool WiFi_IsConnected();