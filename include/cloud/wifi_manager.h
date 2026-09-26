#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "cloud/wifi_store.h"

// -----------------------------------------------------------------------------
// WiFi Manager (Gateway only)
// -----------------------------------------------------------------------------
// Chỉ quản lý kết nối WiFi. Không biết gì về Firebase hay dữ liệu ứng dụng
// - đúng nguyên tắc tách lớp đã áp dụng cho các module khác (can/, uwb/).
//
// Gateway LUÔN phát AP "GATEWAY" + trang cấu hình (wifi_portal). Boot KHÔNG tự
// nối mạng đã lưu - luôn chờ người dùng chọn mạng trên trang web (mạng đã lưu
// chỉ để "kết nối nhanh"). Rớt ngắn thì driver tự nối lại (auto-reconnect).

enum WifiConnState : uint8_t
{
    WIFI_CS_IDLE,       // chưa có yêu cầu nào
    WIFI_CS_CONNECTING,
    WIFI_CS_OK,         // lần nối gần nhất thành công
    WIFI_CS_FAIL,       // lần nối gần nhất thất bại (xem error)
};

struct WifiStatus
{
    WifiConnState state;
    char          error[64]; // lý do khi state == WIFI_CS_FAIL
};

// TẠM THỜI: nối mạng gán cứng WIFI_SSID (STA-only, không AP/portal), blocking
// tối đa WIFI_CONNECT_TIMEOUT_MS. Trả false nếu hết timeout. Dùng THAY WiFi_Start().
bool WiFi_Connect();

// Bật AP + portal + task WiFi, KHÔNG blocking. Gọi 1 lần trong setup().
void WiFi_Start();

bool WiFi_IsConnected();

// Đang có yêu cầu nối mạng chưa xong (chưa nên dùng mạng).
bool WiFi_IsConnecting();

// Tên AP đang phát (ghi vào buf).
void WiFi_GetApSsid(char *buf, size_t bufSize);

// Yêu cầu nối 1 mạng (chạy ở task WiFi). Trả false nếu đang bận nối mạng khác.
// channel: kênh thấy lúc quét (0 = không rõ) - giúp nối nhanh hơn nhiều.
bool WiFi_RequestConnect(const WifiCred &cred, int32_t channel = 0);

void WiFi_GetStatus(WifiStatus &out);

// Quên mạng khỏi NVS; nếu đó là mạng đang nối thì ngắt luôn.
bool WiFi_Forget(const char *ssid);
