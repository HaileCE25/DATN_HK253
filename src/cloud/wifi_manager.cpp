#include "cloud/wifi_manager.h"
#include "cloud/wifi_portal.h"
#include "cloud/cloud_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wpa2.h>
#include <string.h>
#include "shared/config.h"

// -----------------------------------------------------------------------------
// Trạng thái dùng chung giữa task WiFi và các handler HTTP (task async_tcp).
// Mọi truy cập qua s_mu.
// -----------------------------------------------------------------------------
static SemaphoreHandle_t s_mu = nullptr;
static WifiStatus s_status = {WIFI_CS_IDLE, {0}};
static WifiCred s_req;
static int32_t s_reqChannel = 0;
static bool s_pending = false;
static char s_apSsid[32] = {0};

// Lý do rớt gần nhất do driver báo (ghi ở event handler, đọc ở task WiFi).
static volatile uint8_t s_lastReason = 0;

struct Lock
{
    Lock()  { xSemaphoreTake(s_mu, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(s_mu); }
};

static void SetStatus(WifiConnState state, const char *error)
{
    Lock l;
    s_status.state = state;
    strlcpy(s_status.error, error ? error : "", sizeof(s_status.error));
}

// -----------------------------------------------------------------------------
// Dịch mã lý do rớt của driver sang thông báo dễ hiểu.
// -----------------------------------------------------------------------------
// Lỗi xác thực: thử tiếp cũng vô ích. KHÔNG gồm NO_AP_FOUND - một lần quét hụt
// (hotspot điện thoại, AP đang đổi kênh) rất hay gặp, driver tự thử lại được.
static bool IsFatalReason(uint8_t r)
{
    return r == WIFI_REASON_AUTH_FAIL || r == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT ||
           r == WIFI_REASON_HANDSHAKE_TIMEOUT || r == WIFI_REASON_MIC_FAILURE;
}

static const char *ReasonText(uint8_t r, bool enterprise, char *buf, size_t n)
{
    if (r == WIFI_REASON_NO_AP_FOUND)
        return "Không thấy mạng này (ngoài vùng phủ sóng hoặc sai tên)";
    if (IsFatalReason(r))
        return enterprise ? "Sai tài khoản/mật khẩu hoặc mạng từ chối" : "Sai mật khẩu";
    if (r == 0)
        return "Hết thời gian chờ kết nối";
    snprintf(buf, n, "Không kết nối được (mã %u)", (unsigned)r);
    return buf;
}

static void OnWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
        s_lastReason = info.wifi_sta_disconnected.reason;
    else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP)
        s_lastReason = 0;
    // Có thiết bị vào AP -> chỉ đường tới trang cấu hình, MỖI THIẾT BỊ 1 LẦN.
    // Laptop/điện thoại hay tự rời AP (không có Internet) rồi vào lại liên tục,
    // nên chỉ log khi là thiết bị khác thiết bị vừa log.
    else if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED)
    {
        static uint8_t s_lastMac[6] = {0};
        if (memcmp(s_lastMac, info.wifi_ap_staconnected.mac, 6) != 0)
        {
            memcpy(s_lastMac, info.wifi_ap_staconnected.mac, 6);
            LOG_PRINTF("[WIFI] Có thiết bị kết nối vào \"%s\" - mở http://%s để cấu hình WiFi\n",
                       s_apSsid, WiFi.softAPIP().toString().c_str());
        }
    }
}

// -----------------------------------------------------------------------------
// Kết nối
// -----------------------------------------------------------------------------
static void WaitScanIdle()
{
    uint32_t t = millis();
    while (WiFi.scanComplete() == WIFI_SCAN_RUNNING && millis() - t < 8000)
        delay(50);
}

// Nối 1 mạng, blocking tối đa timeoutMs. Trả true nếu có IP. Thất bại: outReason
// là mã driver báo lần cuối (0 = hết giờ mà không có mã).
// channel > 0: kênh lấy từ lần quét trên trang web -> driver chỉ dò đúng kênh đó
// thay vì quét cả 13 kênh (vừa chậm vừa làm AP cấu hình chập chờn).
static bool TryConnect(const WifiCred &c, uint32_t timeoutMs, uint8_t &outReason, int32_t channel = 0)
{
    LOG_PRINTF("[WIFI] Connecting to %s%s", c.ssid, c.enterprise ? " (Enterprise)" : "");

    WiFi.disconnect(); // chỉ ngắt STA, AP vẫn phát
    s_lastReason = 0;

    if (c.enterprise)
    {
        const char *user = c.username;
        const char *ident = c.identity[0] ? c.identity : c.username;
        WiFi.begin(c.ssid, WPA2_AUTH_PEAP, ident, user, c.pass);
    }
    else
    {
        // begin() kiểu Enterprise để lại cờ bật; mạng thường phải tắt tường minh.
        esp_wifi_sta_wpa2_ent_disable();
        WiFi.begin(c.ssid, c.pass[0] ? c.pass : nullptr, channel);
    }

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        // Sai mật khẩu thì báo ngay; không thấy mạng thì để driver thử lại tới hết giờ.
        if (IsFatalReason(s_lastReason) || millis() - start > timeoutMs)
        {
            Serial.println();
            outReason = s_lastReason;
            LOG_PRINTF("[WIFI ERROR] Kết nối thất bại (mã %u)\n", (unsigned)outReason);
            WiFi.disconnect(); // dừng auto-reconnect vào mạng hỏng
            return false;
        }
        Serial.print(".");
        delay(300);
    }

    Serial.println();
    LOG_PRINTF("[WIFI] Connected to %s, IP: %s\n", c.ssid, WiFi.localIP().toString().c_str());
    outReason = 0;
    return true;
}

static uint32_t TimeoutFor(const WifiCred &c)
{
    return c.enterprise ? WIFI_ENTERPRISE_CONNECT_TIMEOUT_MS : WIFI_CONNECT_TIMEOUT_MS;
}

// Nối cred theo yêu cầu từ trang web, cập nhật trạng thái.
static void DoConnect(const WifiCred &cred, int32_t channel)
{
    SetStatus(WIFI_CS_CONNECTING, "");
    WaitScanIdle();

    // Nhớ mạng đang dùng để khôi phục nếu đổi mạng thất bại (Gateway cần
    // Firebase, không nên bị mất kết nối chỉ vì gõ sai mật khẩu mạng khác).
    WifiCred prev = {};
    bool hadPrev = false;
    if (WiFi.status() == WL_CONNECTED)
    {
        String cur = WiFi.SSID();
        if (cur != cred.ssid && WifiStore_Find(cur.c_str(), prev))
            hadPrev = true;
    }

    uint8_t reason = 0;
    if (TryConnect(cred, TimeoutFor(cred), reason, channel))
    {
        WifiStore_Save(cred); // lên đầu danh sách = nút "Kết nối" nhanh trên trang web
        SetStatus(WIFI_CS_OK, "");
        return;
    }

    char buf[48];
    SetStatus(WIFI_CS_FAIL, ReasonText(reason, cred.enterprise, buf, sizeof(buf)));

    if (hadPrev)
    {
        LOG_PRINTF("[WIFI] Khôi phục mạng trước đó: %s\n", prev.ssid);
        uint8_t r2;
        TryConnect(prev, TimeoutFor(prev), r2);
    }
}

// -----------------------------------------------------------------------------
// Task WiFi: xử lý yêu cầu từ trang web, chạy DNS captive portal.
// Boot KHÔNG tự nối mạng đã lưu: luôn chờ người dùng chọn trên trang cấu hình
// (danh sách đã lưu chỉ dùng cho nút "Kết nối" nhanh). AP luôn phát.
// -----------------------------------------------------------------------------
static void WiFiTask(void *)
{
    for (;;)
    {
        WiFiPortal_Process();

        WifiCred req;
        int32_t channel = 0;
        bool have = false;
        {
            Lock l;
            if (s_pending)
            {
                req = s_req;
                channel = s_reqChannel;
                s_pending = false;
                have = true;
            }
        }
        if (have)
            DoConnect(req, channel);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// -----------------------------------------------------------------------------
// API
// -----------------------------------------------------------------------------
bool WiFi_Connect()
{
    if (!s_mu)
        s_mu = xSemaphoreCreateMutex(); // WiFi_IsConnecting() vẫn gọi được

    LOG_PRINTF("[WIFI] Connecting to %s", WIFI_SSID);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true); // driver tự nối lại khi rớt mạng
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t startTime = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT_MS)
        {
            Serial.println();
            LOG_PRINTLN("[WIFI ERROR] Connect timeout");
            return false;
        }
        Serial.print(".");
        delay(300);
    }

    Serial.println();
    LOG_PRINTF("[WIFI] Connected, IP: %s, DNS: %s\n",
               WiFi.localIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
    return true;
}

void WiFi_Start()
{
    s_mu = xSemaphoreCreateMutex();

    strlcpy(s_apSsid, WIFI_AP_SSID, sizeof(s_apSsid));

    WiFi.onEvent(OnWiFiEvent);
    // Không để driver tự lưu/tự nối mạng cũ lúc boot - chỉ nối khi người dùng chọn.
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setAutoReconnect(true); // driver tự nối lại mạng hiện tại khi rớt ngắn
    WiFi.softAP(s_apSsid, WIFI_AP_PASSWORD);
    delay(100);

    LOG_PRINTF("[WIFI] Đang phát WiFi \"%s\" (mật khẩu: %s) - kết nối vào mạng này để cấu hình\n",
               s_apSsid, WIFI_AP_PASSWORD);

    WiFiPortal_Begin();
    xTaskCreatePinnedToCore(WiFiTask, "WiFi", 6144, nullptr, 1, nullptr, 0);
}

bool WiFi_IsConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

bool WiFi_IsConnecting()
{
    Lock l;
    return s_pending || s_status.state == WIFI_CS_CONNECTING;
}

void WiFi_GetApSsid(char *buf, size_t bufSize)
{
    strlcpy(buf, s_apSsid, bufSize);
}

bool WiFi_RequestConnect(const WifiCred &cred, int32_t channel)
{
    Lock l;
    if (s_status.state == WIFI_CS_CONNECTING || s_pending)
        return false;

    s_req = cred;
    s_reqChannel = channel;
    s_pending = true;
    s_status.state = WIFI_CS_CONNECTING; // báo ngay để trang web thấy "đang nối"
    s_status.error[0] = '\0';
    return true;
}

void WiFi_GetStatus(WifiStatus &out)
{
    Lock l;
    out = s_status;
}

bool WiFi_Forget(const char *ssid)
{
    bool removed = WifiStore_Remove(ssid);

    if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid)
    {
        LOG_PRINTF("[WIFI] Quên mạng đang dùng (%s) -> ngắt kết nối\n", ssid);
        WiFi.disconnect();
        SetStatus(WIFI_CS_IDLE, "");
    }
    return removed;
}
