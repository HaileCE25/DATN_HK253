#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Cloud Subsystem Configuration (Gateway only)
// -----------------------------------------------------------------------------

// TẠM THỜI gán cứng mạng WiFi (WiFi_Connect) để test nhanh. Captive portal
// (WiFi_Start, wifi_portal, wifi_store) vẫn giữ nguyên code nhưng chưa dùng.
#define WIFI_SSID       "Mai Thao 2.4GHz"
#define WIFI_PASSWORD   "1141211412"

// Access point cấu hình (chỉ dùng khi chạy captive portal qua WiFi_Start).
// Đổi mật khẩu này (>= 8 ký tự) và dán nhãn lên thiết bị: ai vào được AP này
// thì đổi được WiFi của Gateway.
#define WIFI_AP_SSID         "GATEWAY"
#define WIFI_AP_PASSWORD     "gateway123"

#define FIREBASE_DATABASE_URL  "https://smart-car-rental-b2b-default-rtdb.asia-southeast1.firebasedatabase.app"

// Web API Key - Dùng cho Anonymous Authentication.
#define FIREBASE_API_KEY  "AIzaSyDxuUuzu-L5cippJqnqujOCLztAj-1smEk"

constexpr uint8_t FIREBASE_MASTER_KEY[32] = {
    'M', 'y', 'S', 'u', 'p', 'e', 'r', 'S',
    'e', 'c', 'r', 'e', 't', 'M', 'a', 's',
    't', 'e', 'r', 'K', 'e', 'y', '3', '2',
    'B', 'y', 't', 'e', 's', '!', '!', '!'
};

// Timeout chờ WiFi connect trước khi coi là fail.
#define WIFI_CONNECT_TIMEOUT_MS             15000
// WPA2-Enterprise xác thực nhiều bước nên lâu hơn.
#define WIFI_ENTERPRISE_CONNECT_TIMEOUT_MS  25000