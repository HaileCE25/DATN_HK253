#include "cloud/firebase_manager.h"
#include "cloud/cloud_config.h"
#include "cloud/wifi_manager.h"
#include "shared/config.h"
#include <Firebase_ESP_Client.h>

static FirebaseData s_fbdo;
static FirebaseAuth s_auth;
static FirebaseConfig s_config;
static bool s_ready = false;

bool Firebase_Init()
{
    if (s_ready)
        return true;

    if (!WiFi_IsConnected())
    {
        LOG_PRINTLN("[FIREBASE ERROR] WiFi not connected");
        return false;
    }

    s_config.api_key = FIREBASE_API_KEY;
    s_config.database_url = FIREBASE_DATABASE_URL;

    // Anonymous Auth: signUp() với email/password rỗng == đăng nhập ẩn
    // danh, nếu tính năng Anonymous đã được bật trong Firebase Console
    // (Authentication -> Sign-in method -> Anonymous).
    if (!Firebase.signUp(&s_config, &s_auth, "", ""))
    {
        LOG_PRINTF("[FIREBASE ERROR] Anonymous sign-in failed: %s\n",
                   s_config.signer.signupError.message.c_str());
        return false;
    }

    Firebase.begin(&s_config, &s_auth);
    Firebase.reconnectWiFi(true);

    s_ready = true;
    LOG_PRINTLN("[FIREBASE] Init OK (anonymous auth)");
    return true;
}

bool Firebase_IsReady()
{
    return s_ready;
}

bool Firebase_ReadString(const char* path, String& outValue)
{
    if (!s_ready)
        return false;

    if (Firebase.RTDB.getString(&s_fbdo, path))
    {
        outValue = s_fbdo.stringData();
        return true;
    }

    // Path không tồn tại không phải lỗi kết nối - có thể là field tuỳ chọn
    // (vd. expire_time). Nơi gọi tự quyết định có log hay không.
    if (s_fbdo.httpCode() != FIREBASE_ERROR_PATH_NOT_EXIST)
        LOG_PRINTF("[FIREBASE ERROR] Read failed at %s: %s\n", path, s_fbdo.errorReason().c_str());
    return false;
}

bool Firebase_QueryEqualTo(const char* path, const char* child, const char* value, String& outJson)
{
    if (!s_ready)
        return false;

    QueryFilter query;
    query.orderBy(child);
    query.equalTo(value);

    bool ok = Firebase.RTDB.getJSON(&s_fbdo, path, &query);
    query.clear();

    if (ok)
    {
        outJson = s_fbdo.payload();
        return true;
    }

    // Path chưa tồn tại: RTDB trả null, thư viện báo lỗi "type mismatch" nhưng
    // thực chất là "không có node nào khớp".
    if (s_fbdo.httpCode() == 200 && s_fbdo.dataTypeEnum() == firebase_rtdb_data_type_null)
    {
        outJson = "{}";
        return true;
    }

    LOG_PRINTF("[FIREBASE ERROR] Query failed at %s (%s == %s): %s\n",
               path, child, value, s_fbdo.errorReason().c_str());
    return false;
}

bool Firebase_WriteString(const char* path, const String& value)
{
    if (!s_ready)
        return false;

    if (Firebase.RTDB.setString(&s_fbdo, path, value))
    {
        return true;
    }

    LOG_PRINTF("[FIREBASE ERROR] Write failed at %s: %s\n", path, s_fbdo.errorReason().c_str());
    return false;
}

bool Firebase_WriteBool(const char* path, bool value)
{
    if (!s_ready)
        return false;

    if (Firebase.RTDB.setBool(&s_fbdo, path, value))
    {
        return true;
    }

    LOG_PRINTF("[FIREBASE ERROR] Write failed at %s: %s\n", path, s_fbdo.errorReason().c_str());
    return false;
}