#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "mbedtls/aes.h"

#include "can/twai_driver.h"
#include "can/isotp.h"
#include "can/payloads.h"
#include "can/can_ids.h"
#include "cloud/wifi_manager.h"
#include "cloud/firebase_manager.h"
#include "cloud/cloud_config.h"
#include "shared/config.h"
#include "shared/vn_time.h"

constexpr uint32_t ISOTP_TIMEOUT_MS = 2000;
constexpr uint32_t CAN_DISPATCH_POLL_MS = 100;
// Khởi tạo Firebase thất bại: thử lại sau 5 s, nhân đôi mỗi lần, tối đa 5 phút.
constexpr uint32_t FIREBASE_RETRY_MIN_MS = 5000;
constexpr uint32_t FIREBASE_RETRY_MAX_MS = 300000;

static bool s_canReady = false;

static void PrintHex(const char* label, const uint8_t* data, size_t length)
{
    LOG_PRINTF("%s ", label);
    for (size_t i = 0; i < length; i++)
    {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

static bool HexStringToBytes(const String& hex, uint8_t* outBytes, size_t maxBytes, size_t& outLen)
{
    size_t hexLen = hex.length();
    if (hexLen % 2 != 0)
        return false;

    size_t byteLen = hexLen / 2;
    if (byteLen > maxBytes)
        return false;

    for (size_t i = 0; i < byteLen; i++)
    {
        char high = hex.charAt(i * 2);
        char low = hex.charAt(i * 2 + 1);

        auto hexDigit = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };

        int h = hexDigit(high);
        int l = hexDigit(low);
        if (h < 0 || l < 0)
            return false;

        outBytes[i] = (uint8_t)((h << 4) | l);
    }

    outLen = byteLen;
    return true;
}

static bool AesCtrDecrypt(
    const uint8_t* key32,
    const uint8_t* iv16,
    const uint8_t* ciphertext,
    size_t ctLen,
    uint8_t* outPlaintext)
{
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    if (mbedtls_aes_setkey_enc(&aes, key32, 256) != 0)
    {
        mbedtls_aes_free(&aes);
        return false;
    }

    uint8_t nonceCounter[16];
    memcpy(nonceCounter, iv16, 16);
    uint8_t streamBlock[16] = {0};
    size_t ncOff = 0;

    int ret = mbedtls_aes_crypt_ctr(
        &aes, ctLen, &ncOff, nonceCounter, streamBlock, ciphertext, outPlaintext);

    mbedtls_aes_free(&aes);
    return ret == 0;
}

// Đổi "YYYY-MM-DD HH:MM" (giờ Việt Nam, UTC+7) sang Unix time UTC.
// Chấp nhận thêm ":SS" ở cuối. Trả false nếu sai định dạng / ngoài khoảng hợp lý.
static bool ParseExpireVietnamTime(const String& text, uint32_t& outUnix)
{
    // Bỏ qua khoảng trắng / dấu ngoặc kép thừa ở đầu (nhập tay trên Console
    // dễ gõ cả dấu "), ký tự thừa ở cuối do sscanf tự bỏ qua.
    const char* p = text.c_str();
    while (*p && !(*p >= '0' && *p <= '9'))
        p++;

    int year, month, day, hour, minute;
    if (sscanf(p, "%d-%d-%d %d:%d", &year, &month, &day, &hour, &minute) != 5)
        return false;

    if (year < 2020 || year > 2100 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59)
        return false;

    // days_from_civil (Howard Hinnant): số ngày kể từ 1970-01-01.
    int y = year - (month <= 2 ? 1 : 0);
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097 + doe - 719468;

    int64_t unixUtc = days * 86400 + hour * 3600 + minute * 60 - 7 * 3600;
    if (unixUtc <= 0 || unixUtc > 0xFFFFFFFFLL)
        return false;

    outUnix = (uint32_t)unixUtc;
    return true;
}

// =============================================================================
//  KIỂM TRA BOOKING — cơ chế thu hồi key
// =============================================================================
// Xe chỉ được cấp key khi có ít nhất 1 booking status == "ACTIVE". Admin thu
// hồi bằng cách đổi status sang "REVOKED"; Car hỏi lại định kỳ (xem
// KEY_REVALIDATE_INTERVAL_MS phía Car) nên sẽ nhận KEY_STATUS_REVOKED và huỷ key.
enum BookingCheckResult
{
    BOOKING_ACTIVE,
    BOOKING_NOT_ACTIVE,     // tra được, không có booking ACTIVE -> thu hồi
    BOOKING_LOOKUP_FAILED,  // không tra được -> không kết luận gì
};

static const char* BOOKING_STATUS_ACTIVE = "ACTIVE";

// verbose = false (Car hỏi lại định kỳ): chỉ in lỗi và kết quả thu hồi.
static BookingCheckResult CheckBookingActive(const char* carId, bool verbose)
{
    String json;
    if (!Firebase_QueryEqualTo("/Bookings", "car_id", carId, json))
        return BOOKING_LOOKUP_FAILED;

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err)
    {
        LOG_PRINTF("[GATEWAY ERROR] Bookings trả JSON lỗi: %s\n", err.c_str());
        return BOOKING_LOOKUP_FAILED;
    }

    const char* lastStatus = nullptr;
    for (JsonPair booking : doc.as<JsonObject>())
    {
        const char* status = booking.value()["status"] | "";
        if (strcmp(status, BOOKING_STATUS_ACTIVE) == 0)
        {
            if (verbose)
                LOG_PRINTF("[GATEWAY] Booking %s ACTIVE cho car_id=%s\n", booking.key().c_str(), carId);
            return BOOKING_ACTIVE;
        }
        lastStatus = status;
    }

    LOG_PRINTF("[GATEWAY] car_id=%s không có booking ACTIVE (%s)\n",
               carId, lastStatus ? lastStatus : "không có booking nào");
    return BOOKING_NOT_ACTIVE;
}

static void SendKeyResponse(const KeyResponsePayload& response, const char* carId, bool verbose)
{
    uint8_t respBuf[KEY_RESPONSE_PAYLOAD_SIZE];
    if (!SerializeKeyResponse(response, respBuf, sizeof(respBuf)))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Serialize KeyResponse thất bại");
        return;
    }

    if (!ISOTP_Send(CAN_ID_KEY_PROVISION_RESP, respBuf, sizeof(respBuf), ISOTP_TIMEOUT_MS))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Gửi KeyResponse qua CAN thất bại");
        return;
    }

    if (response.status == KEY_STATUS_REVOKED)
        LOG_PRINTF("[GATEWAY] Đã báo thu hồi Keyroot qua CAN cho car_id=%s\n", carId);
    else if (verbose)
        LOG_PRINTF("[GATEWAY] Đã giải mã và gửi Keyroot (%u byte) qua CAN cho car_id=%s\n",
                   (unsigned)response.key_len, carId);
}

// =============================================================================
//  XỬ LÝ CAN_ID_KEY_PROVISION_REQ — lấy key_root từ Firebase, trả qua CAN
// =============================================================================
static void HandleKeyRequest(const twai_message_t& firstFrame)
{
    uint8_t reqBuf[KEY_REQUEST_PAYLOAD_SIZE];
    size_t reqLen = 0;

    if (!ISOTP_ReceiveFromFirstFrame(firstFrame, reqBuf, sizeof(reqBuf), reqLen, ISOTP_TIMEOUT_MS))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Nhan KeyRequest ISO-TP that bai");
        return;
    }

    KeyRequestPayload request;
    if (!DeserializeKeyRequest(reqBuf, reqLen, request))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Invalid KeyRequest payload");
        return;
    }

    // Lượt Car hỏi lại định kỳ (~10 s/lần): xử lý y hệt nhưng không in log/debug.
    const bool verbose = !request.revalidate;

    if (verbose)
        LOG_PRINTF("[GATEWAY] Yêu cầu xin Keyroot nhận được qua CAN, car_id=%s\n", request.car_id);

    // Kiểm tra booking TRƯỚC khi đụng tới key - xe bị thu hồi thì không giải mã.
    switch (CheckBookingActive(request.car_id, verbose))
    {
    case BOOKING_LOOKUP_FAILED:
        LOG_PRINTF("[GATEWAY ERROR] Không tìm thấy Keyroot cho car_id=%s\n", request.car_id);
        return;

    case BOOKING_NOT_ACTIVE:
    {
        KeyResponsePayload revoked = {};
        revoked.expire_unix = KEY_EXPIRE_UNKNOWN;
        revoked.status = KEY_STATUS_REVOKED;
        SendKeyResponse(revoked, request.car_id, verbose);
        return;
    }

    case BOOKING_ACTIVE:
        break;
    }

    String basePath = String("/SecureKeys/") + request.car_id;
    String ivHex, encryptedHex;

    if (!Firebase_ReadString((basePath + "/iv").c_str(), ivHex) ||
        !Firebase_ReadString((basePath + "/encrypted_key_root").c_str(), encryptedHex))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Firebase lookup thất bại");
        return;
    }

    uint8_t iv[16];
    size_t ivLen = 0;
    if (!HexStringToBytes(ivHex, iv, sizeof(iv), ivLen) || ivLen != 16)
    {
        LOG_PRINTLN("[GATEWAY ERROR] IV không hợp lệ");
        return;
    }

    uint8_t ciphertext[64];
    size_t ctLen = 0;
    if (!HexStringToBytes(encryptedHex, ciphertext, sizeof(ciphertext), ctLen))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Keyroot sau khi mã hóa không đúng định dạng hex");
        return;
    }

    uint8_t plaintext[64] = {0};
    if (!AesCtrDecrypt(FIREBASE_MASTER_KEY, iv, ciphertext, ctLen, plaintext))
    {
        LOG_PRINTLN("[GATEWAY ERROR] AES decrypt thất bại");
        return;
    }

    String keyRootHex;
    keyRootHex.reserve(ctLen);
    for (size_t i = 0; i < ctLen; i++)
    {
        keyRootHex += (char)plaintext[i];
    }

    KeyResponsePayload response = {};
    size_t keyLen = 0;

    if (!HexStringToBytes(keyRootHex, response.key_root, sizeof(response.key_root), keyLen))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Sau khi giải mã, Keyroot không đúng định dạng hex");
        return;
    }

    response.key_len = (uint8_t)keyLen;
    response.status = KEY_STATUS_OK;

    // expire_time không bắt buộc: thiếu/sai định dạng thì vẫn cấp key, gửi
    // KEY_EXPIRE_UNKNOWN để Car biết là không có thông tin hạn.
    response.expire_unix = KEY_EXPIRE_UNKNOWN;
    String expireText;
    if (!Firebase_ReadString((basePath + "/expire_time").c_str(), expireText))
    {
        if (verbose) LOG_PRINTLN("[GATEWAY WARN] Không đọc được expire_time trên Firebase");
    }
    else if (!ParseExpireVietnamTime(expireText, response.expire_unix))
    {
        response.expire_unix = KEY_EXPIRE_UNKNOWN;
        if (verbose) LOG_PRINTF("[GATEWAY WARN] expire_time sai định dạng: \"%s\"\n", expireText.c_str());
    }
    else if (verbose)
    {
        char expireVn[24];
        VnTime_Format(response.expire_unix, expireVn, sizeof(expireVn));
        LOG_PRINTF("[GATEWAY] expire_time \"%s\" -> %s (gio Viet Nam, unix %lu)\n",
                   expireText.c_str(), expireVn, (unsigned long)response.expire_unix);
    }

    if (verbose)
        PrintHex("[GATEWAY DEBUG] Keyroot sau khi giải mã:", response.key_root, response.key_len);

    SendKeyResponse(response, request.car_id, verbose);
}

// =============================================================================
//  XỬ LÝ CAN_ID_ACTUATOR_CMD — nhận lệnh mở/khoá từ ECU Access, kích relay
// =============================================================================
// static void HandleActuatorCommand(const twai_message_t& firstFrame)
// {
//     uint8_t cmdBuf[1];
//     size_t cmdLen = 0;

//     // Payload 1 byte luôn là Single Frame -> không cần chờ thêm frame nào
//     if (!ISOTP_ReceiveFromFirstFrame(firstFrame, cmdBuf, sizeof(cmdBuf), cmdLen, 0))
//     {
//         LOG_PRINTLN("[GATEWAY ERROR] ActuatorCmd: frame khong hop le");
//         return;
//     }

//     if (cmdLen < 1)
//     {
//         LOG_PRINTLN("[GATEWAY ERROR] ActuatorCmd: payload rỗng");
//         return;
//     }

//     switch (cmdBuf[0])
//     {
//     case ACTUATOR_CMD_UNLOCK:
//         LOG_PRINTLN("[GATEWAY ACTUATOR] UNLOCK - kich relay mo khoa");
//         // TODO: digitalWrite(RELAY_PIN, HIGH) hoặc tín hiệu tương đương
//         break;

//     case ACTUATOR_CMD_LOCK:
//         LOG_PRINTLN("[GATEWAY ACTUATOR] LOCK - tat relay, khoa lai");
//         // TODO: digitalWrite(RELAY_PIN, LOW)
//         break;

//     default:
//         LOG_PRINTF("[GATEWAY ERROR] ActuatorCmd unknown: 0x%02X\n", cmdBuf[0]);
//         break;
//     }
// }

// =============================================================================
//  SETUP + LOOP
// =============================================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    LOG_PRINTLN("===============================");
    LOG_PRINTLN("   GATEWAY START");
    LOG_PRINTLN("===============================");

    s_canReady = TWAI_Init();

    // Không blocking: bật AP + trang cấu hình, nối lại mạng dùng gần nhất trong
    // task WiFi. Firebase được khởi tạo trong loop() khi đã có WiFi.
    WiFi_Start();

    LOG_PRINTLN("[GATEWAY] Chờ kết nối tới Firebase và yêu cầu cấp khóa từ Car qua CAN");
}

// Vòng dispatch CAN DUY NHẤT: đọc 1 frame rồi route theo identifier.
// Không để từng handler tự đọc bus - ISOTP_Receive(id) vứt bỏ mọi frame khác
// ID trong lúc chờ, nên handler này sẽ "ăn mất" frame của handler kia.
void loop()
{
    // Firebase cần WiFi ra được Internet. WiFi do task WiFi lo (người dùng chọn
    // trên trang cấu hình), ở đây chỉ chờ mạng ổn định rồi khởi tạo 1 lần.
    // Mỗi lần thử có thể chặn loop() vài giây (TLS timeout) nên lùi dần thời
    // gian thử lại; đổi sang mạng khác thì thử lại ngay.
    static uint32_t  s_fbNextTryMs = 0;
    static uint32_t  s_fbBackoffMs = FIREBASE_RETRY_MIN_MS;
    static IPAddress s_fbLastIp;
    if (!Firebase_IsReady() && WiFi_IsConnected() && !WiFi_IsConnecting())
    {
        IPAddress ip = WiFi.localIP();
        if (ip != s_fbLastIp)
        {
            s_fbLastIp    = ip;
            s_fbBackoffMs = FIREBASE_RETRY_MIN_MS;
            s_fbNextTryMs = 0;
        }

        if (s_fbNextTryMs == 0 || (int32_t)(millis() - s_fbNextTryMs) >= 0)
        {
            if (Firebase_Init())
            {
                LOG_PRINTLN("[GATEWAY] Sẵn sàng, chờ yêu cầu khóa từ Car qua CAN");
            }
            else
            {
                s_fbNextTryMs = (millis() + s_fbBackoffMs) | 1;
                s_fbBackoffMs = min(s_fbBackoffMs * 2, FIREBASE_RETRY_MAX_MS);
            }
        }
    }

    if (!s_canReady)
    {
        delay(1000); // TWAI init lỗi: TWAI_Receive trả false ngay -> tránh busy-loop
        return;
    }

    twai_message_t frame;
    if (!TWAI_Receive(frame, pdMS_TO_TICKS(CAN_DISPATCH_POLL_MS)))
        return;

    if (frame.extd || frame.rtr)
        return;

    switch (frame.identifier)
    {
    case CAN_ID_KEY_PROVISION_REQ: HandleKeyRequest(frame);      break;
    // Tạm bỏ actuator: frame 0x200 rơi vào default và bị bỏ qua.
    // case CAN_ID_ACTUATOR_CMD:      HandleActuatorCommand(frame); break;
    default:                                                     break;
    }
}
