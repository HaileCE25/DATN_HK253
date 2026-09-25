#include "cloud/wifi_portal.h"
#include "cloud/wifi_manager.h"
#include "cloud/wifi_store.h"
#include "cloud/cloud_config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <algorithm>
#include <vector>
#include "shared/config.h"

// Trang web nằm ở web/index.html, được nhúng vào firmware lúc build
// (board_build.embed_txtfiles trong platformio.ini) và kết thúc bằng '\0'.
extern const char WEB_INDEX_HTML[] asm("_binary_web_index_html_start");

// Các handler chạy trong task async_tcp -> KHÔNG được block. Việc nối WiFi chỉ
// được yêu cầu qua WiFi_RequestConnect(), task WiFi mới thực sự nối.

static AsyncWebServer s_server(80);
static DNSServer s_dns;

struct ScanEntry
{
    String ssid;
    int    rssi;
    bool   sec;
    bool   ent;
    int    ch;
    wifi_auth_mode_t auth;
};

// Cache kết quả quét. Chỉ handler HTTP đụng vào (cùng 1 task async_tcp nên
// không cần khoá).
static std::vector<ScanEntry> s_scan;
static bool s_scanned = false;
static uint32_t s_scanStartedMs = 0;

// Dedupe theo SSID (giữ RSSI mạnh nhất), bỏ SSID rỗng, sắp giảm dần theo RSSI.
static void CollectScan(int count)
{
    s_scan.clear();
    for (int i = 0; i < count; i++)
    {
        String s = WiFi.SSID(i);
        if (s.length() == 0)
            continue;

        bool dup = false;
        for (int j = 0; j < count && !dup; j++)
        {
            if (j != i && WiFi.SSID(j) == s &&
                (WiFi.RSSI(j) > WiFi.RSSI(i) || (WiFi.RSSI(j) == WiFi.RSSI(i) && j < i)))
                dup = true;
        }
        if (dup)
            continue;

        ScanEntry e;
        e.ssid = s;
        e.rssi = WiFi.RSSI(i);
        e.sec = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
        e.ent = WiFi.encryptionType(i) == WIFI_AUTH_WPA2_ENTERPRISE;
        e.ch = WiFi.channel(i);
        e.auth = WiFi.encryptionType(i);
        s_scan.push_back(e);
    }
    std::sort(s_scan.begin(), s_scan.end(),
              [](const ScanEntry &a, const ScanEntry &b) { return a.rssi > b.rssi; });
    s_scanned = true;
}

static void SendJson(AsyncWebServerRequest *req, int code, JsonDocument &doc)
{
    String out;
    serializeJson(doc, out);
    req->send(code, "application/json", out);
}

static const char *StateName(WifiConnState s)
{
    switch (s)
    {
    case WIFI_CS_CONNECTING: return "connecting";
    case WIFI_CS_OK:         return "ok";
    case WIFI_CS_FAIL:       return "fail";
    default:                 return "idle";
    }
}

// Tên kiểu bảo mật hiển thị trên trang web.
static const char *AuthName(wifi_auth_mode_t a)
{
    switch (a)
    {
    case WIFI_AUTH_OPEN:            return "Mạng mở";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3-SAE";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "Bảo mật";
    }
}

// GET /api/state
static void HandleState(AsyncWebServerRequest *req)
{
    WifiStatus st;
    WiFi_GetStatus(st);

    char ap[32];
    WiFi_GetApSsid(ap, sizeof(ap));

    JsonDocument doc;
    doc["ap"] = ap;
    doc["state"] = StateName(st.state);
    doc["error"] = st.error;
    bool connected = WiFi_IsConnected();
    doc["connected"] = connected;
    if (connected)
    {
        doc["ssid"] = WiFi.SSID();
        doc["ip"] = WiFi.localIP().toString();
        doc["rssi"] = WiFi.RSSI();
    }
    SendJson(req, 200, doc);
}

// GET /api/scan[?rescan=1] - danh sách mạng thấy được, gộp với mạng đã lưu
static void HandleScan(AsyncWebServerRequest *req)
{
    WifiStatus st;
    WiFi_GetStatus(st);
    bool connecting = st.state == WIFI_CS_CONNECTING;

    // Quét khi đang nối mạng sẽ phá kết nối -> chỉ trả cache trong lúc đó.
    int r = WiFi.scanComplete();
    if (r >= 0)
    {
        CollectScan(r);
        WiFi.scanDelete();
        r = WIFI_SCAN_FAILED;
    }

    bool want = req->hasParam("rescan") || !s_scanned;
    // Chống bấm dồn: quét khiến STA rời kênh, ảnh hưởng kết nối Firebase.
    bool cooled = millis() - s_scanStartedMs > 5000 || s_scanStartedMs == 0;
    if (!connecting && r != WIFI_SCAN_RUNNING && want && cooled)
    {
        WiFi.scanNetworks(true); // async, không block
        s_scanStartedMs = millis();
        r = WIFI_SCAN_RUNNING;
    }
    bool scanning = (r == WIFI_SCAN_RUNNING);

    WifiCred saved[WIFI_STORE_MAX];
    size_t nSaved = WifiStore_Load(saved);
    String current = WiFi_IsConnected() ? WiFi.SSID() : String();

    JsonDocument doc;
    doc["scanning"] = scanning;
    JsonArray arr = doc["networks"].to<JsonArray>();

    auto isSaved = [&](const String &ssid, size_t *idx) {
        for (size_t i = 0; i < nSaved; i++)
            if (ssid == saved[i].ssid)
            {
                if (idx) *idx = i;
                return true;
            }
        return false;
    };

    for (const ScanEntry &e : s_scan)
    {
        JsonObject o = arr.add<JsonObject>();
        size_t idx = 0;
        bool sv = isSaved(e.ssid, &idx);
        o["ssid"] = e.ssid;
        o["rssi"] = e.rssi;
        o["sec"] = e.sec;
        o["ent"] = e.ent || (sv && saved[idx].enterprise);
        o["ch"] = e.ch;
        o["auth"] = AuthName(e.auth);
        o["saved"] = sv;
        o["current"] = (e.ssid == current);
    }

    SendJson(req, 200, doc);
}

static String Param(AsyncWebServerRequest *req, const char *name)
{
    return req->hasParam(name, true) ? req->getParam(name, true)->value() : String();
}

// POST /api/connect
//   use_saved=1&ssid=...                       nối bằng thông tin đã lưu
//   ssid=...&pass=...                          mạng thường
//   ssid=...&eap=1&identity=..&username=..&pass=..   mạng Enterprise
static void HandleConnect(AsyncWebServerRequest *req)
{
    JsonDocument doc;
    String ssid = Param(req, "ssid");
    if (ssid.length() == 0 || ssid.length() > 32)
    {
        doc["ok"] = false;
        doc["error"] = "bad_ssid";
        SendJson(req, 400, doc);
        return;
    }

    WifiCred cred = {};
    if (Param(req, "use_saved") == "1")
    {
        if (!WifiStore_Find(ssid.c_str(), cred))
        {
            doc["ok"] = false;
            doc["error"] = "not_saved";
            SendJson(req, 404, doc);
            return;
        }
    }
    else
    {
        String pass = Param(req, "pass");
        String identity = Param(req, "identity");
        String username = Param(req, "username");
        bool eap = Param(req, "eap") == "1";

        if (pass.length() > 64 || identity.length() > 64 || username.length() > 64 ||
            (eap && username.length() == 0))
        {
            doc["ok"] = false;
            doc["error"] = "bad_input";
            SendJson(req, 400, doc);
            return;
        }

        strlcpy(cred.ssid, ssid.c_str(), sizeof(cred.ssid));
        strlcpy(cred.pass, pass.c_str(), sizeof(cred.pass));
        cred.enterprise = eap;
        strlcpy(cred.identity, identity.c_str(), sizeof(cred.identity));
        strlcpy(cred.username, username.c_str(), sizeof(cred.username));
    }

    // Kênh từ lần quét gần nhất -> driver không phải dò lại cả 13 kênh.
    int32_t channel = 0;
    for (const ScanEntry &e : s_scan)
        if (e.ssid == cred.ssid) { channel = e.ch; break; }

    if (!WiFi_RequestConnect(cred, channel))
    {
        doc["ok"] = false;
        doc["error"] = "busy";
        SendJson(req, 409, doc);
        return;
    }

    doc["ok"] = true;
    SendJson(req, 200, doc);
}

// POST /api/forget  ssid=...
static void HandleForget(AsyncWebServerRequest *req)
{
    JsonDocument doc;
    String ssid = Param(req, "ssid");
    doc["ok"] = ssid.length() > 0 && WiFi_Forget(ssid.c_str());
    SendJson(req, 200, doc);
}

void WiFiPortal_Begin()
{
    s_dns.start(53, "*", WiFi.softAPIP());

    // Web server nghe trên MỌI interface. ON_AP_FILTER chặn request đến từ mạng
    // STA (vd. người khác cùng mạng trường) - chỉ ai ở trên AP mới đổi được WiFi.
    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
                { req->send(200, "text/html; charset=utf-8", WEB_INDEX_HTML); })
        .setFilter(ON_AP_FILTER);
    s_server.on("/api/state", HTTP_GET, HandleState).setFilter(ON_AP_FILTER);
    s_server.on("/api/scan", HTTP_GET, HandleScan).setFilter(ON_AP_FILTER);
    s_server.on("/api/connect", HTTP_POST, HandleConnect).setFilter(ON_AP_FILTER);
    s_server.on("/api/forget", HTTP_POST, HandleForget).setFilter(ON_AP_FILTER);

    // Captive portal: mọi URL lạ (generate_204, hotspot-detect.html, ncsi.txt...)
    // chuyển về trang cấu hình để điện thoại tự bật popup.
    s_server.onNotFound([](AsyncWebServerRequest *req)
                        {
                            if (!ON_AP_FILTER(req))
                            {
                                req->send(404);
                                return;
                            }
                            req->redirect(String("http://") + WiFi.softAPIP().toString() + "/");
                        });

    s_server.begin();
}

void WiFiPortal_Process()
{
    s_dns.processNextRequest();
}
