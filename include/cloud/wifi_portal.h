#pragma once

// -----------------------------------------------------------------------------
// WiFi Portal (Gateway only)
// -----------------------------------------------------------------------------
// Web server cấu hình WiFi chạy trên AP của Gateway (do wifi_manager bật).
// Trang web: web/index.html (nhúng vào firmware lúc build, xem platformio.ini).
// Chỉ làm HTTP + DNS captive; việc nối mạng thật do wifi_manager thực hiện.

// Đăng ký route, bật web server (chỉ phục vụ client trên AP) và DNS captive.
// Gọi sau khi AP đã phát.
void WiFiPortal_Begin();

// Xử lý DNS captive. Gọi thường xuyên (vài chục ms) từ task WiFi.
void WiFiPortal_Process();
