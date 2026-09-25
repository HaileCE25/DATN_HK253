#include "cloud/wifi_store.h"
#include <Preferences.h>
#include <string.h>

static const char *NVS_NS = "wifi";

static void Key(char *out, size_t n, char prefix, size_t i)
{
    snprintf(out, n, "%c%u", prefix, (unsigned)i);
}

// putString("") không tạo key, nên getString() trên key đó in lỗi NOT_FOUND.
// Kiểm tra trước để không spam log mỗi lần đọc danh sách.
static String GetStr(Preferences &prefs, const char *k)
{
    return prefs.isKey(k) ? prefs.getString(k, "") : String();
}

size_t WifiStore_Load(WifiCred out[WIFI_STORE_MAX])
{
    Preferences prefs;
    // Mở read-write: chế độ read-only báo lỗi NOT_FOUND khi namespace chưa
    // được tạo (lần boot đầu, chưa từng lưu WiFi).
    if (!prefs.begin(NVS_NS, false))
        return 0;

    size_t n = prefs.getUChar("n", 0);
    if (n > WIFI_STORE_MAX)
        n = WIFI_STORE_MAX;

    size_t count = 0;
    for (size_t i = 0; i < n; i++)
    {
        char k[4];
        Key(k, sizeof(k), 's', i);
        String s = GetStr(prefs, k);
        if (s.length() == 0 || s.length() > 32)
            continue;

        WifiCred &c = out[count];
        memset(&c, 0, sizeof(c));
        strlcpy(c.ssid, s.c_str(), sizeof(c.ssid));

        Key(k, sizeof(k), 'p', i);
        strlcpy(c.pass, GetStr(prefs, k).c_str(), sizeof(c.pass));
        Key(k, sizeof(k), 'e', i);
        c.enterprise = prefs.getUChar(k, 0) != 0;
        Key(k, sizeof(k), 'i', i);
        strlcpy(c.identity, GetStr(prefs, k).c_str(), sizeof(c.identity));
        Key(k, sizeof(k), 'u', i);
        strlcpy(c.username, GetStr(prefs, k).c_str(), sizeof(c.username));
        count++;
    }

    prefs.end();
    return count;
}

bool WifiStore_Find(const char *ssid, WifiCred &out)
{
    WifiCred list[WIFI_STORE_MAX];
    size_t n = WifiStore_Load(list);
    for (size_t i = 0; i < n; i++)
    {
        if (strcmp(list[i].ssid, ssid) == 0)
        {
            out = list[i];
            return true;
        }
    }
    return false;
}

// Ghi đè toàn bộ danh sách (theo thứ tự đã cho).
static bool WriteAll(const WifiCred *list, size_t n)
{
    Preferences prefs;
    if (!prefs.begin(NVS_NS, false))
        return false;

    prefs.clear();
    bool ok = true;
    for (size_t i = 0; i < n; i++)
    {
        char k[4];
        Key(k, sizeof(k), 's', i);
        ok &= prefs.putString(k, list[i].ssid) > 0;
        // Chuỗi rỗng (mạng mở / không có identity) trả 0 nhưng không phải lỗi.
        Key(k, sizeof(k), 'p', i);
        prefs.putString(k, list[i].pass);
        Key(k, sizeof(k), 'e', i);
        prefs.putUChar(k, list[i].enterprise ? 1 : 0);
        Key(k, sizeof(k), 'i', i);
        prefs.putString(k, list[i].identity);
        Key(k, sizeof(k), 'u', i);
        prefs.putString(k, list[i].username);
    }
    prefs.putUChar("n", (uint8_t)n);

    prefs.end();
    return ok;
}

bool WifiStore_Save(const WifiCred &cred)
{
    if (cred.ssid[0] == '\0')
        return false;

    WifiCred old[WIFI_STORE_MAX];
    size_t n = WifiStore_Load(old);

    WifiCred merged[WIFI_STORE_MAX];
    size_t m = 0;
    merged[m++] = cred; // mạng mới/vừa dùng lên đầu
    for (size_t i = 0; i < n && m < WIFI_STORE_MAX; i++)
    {
        if (strcmp(old[i].ssid, cred.ssid) != 0)
            merged[m++] = old[i];
    }
    return WriteAll(merged, m);
}

bool WifiStore_Remove(const char *ssid)
{
    WifiCred old[WIFI_STORE_MAX];
    size_t n = WifiStore_Load(old);

    WifiCred kept[WIFI_STORE_MAX];
    size_t k = 0;
    for (size_t i = 0; i < n; i++)
    {
        if (strcmp(old[i].ssid, ssid) != 0)
            kept[k++] = old[i];
    }
    if (k == n)
        return false;
    return WriteAll(kept, k);
}

void WifiStore_Clear()
{
    Preferences prefs;
    if (prefs.begin(NVS_NS, false))
    {
        prefs.clear();
        prefs.end();
    }
}
