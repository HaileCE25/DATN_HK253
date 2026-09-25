#pragma once

#include <stdbool.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// WiFi Store (Gateway only)
// -----------------------------------------------------------------------------
// Danh sách WiFi đã lưu trong NVS, dùng để "kết nối nhanh" trên trang cấu hình.
// Phần tử đầu = mạng nối thành công gần nhất (MRU) - cũng là mạng duy nhất
// Gateway tự nối lại khi boot. Đầy thì mạng cuối (cũ nhất) bị đẩy ra.

constexpr size_t WIFI_STORE_MAX = 5;

struct WifiCred
{
    char ssid[33];
    char pass[65];
    bool enterprise;    // WPA2-Enterprise (PEAP/MSCHAPv2): dùng identity + username + pass
    char identity[65];  // anonymous identity (có thể rỗng -> dùng username)
    char username[65];
};

// Đọc danh sách đã lưu vào out (tối đa WIFI_STORE_MAX), trả số phần tử.
size_t WifiStore_Load(WifiCred out[WIFI_STORE_MAX]);

// Tìm theo SSID. Trả false nếu chưa lưu.
bool WifiStore_Find(const char *ssid, WifiCred &out);

// Thêm hoặc cập nhật mạng, đưa lên đầu danh sách.
bool WifiStore_Save(const WifiCred &cred);

// Quên 1 mạng. Trả false nếu không có.
bool WifiStore_Remove(const char *ssid);

// Xoá toàn bộ.
void WifiStore_Clear();
