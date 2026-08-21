#include <Arduino.h>
#include <Preferences.h>
#include "protocol/packet.h"
#include "car/ble_car.h"
#include "os/queue.h"
#include "crypto/hmac.h"
#include "can/twai_driver.h"
#include "can/isotp.h"
#include "can/payloads.h"
#include "can/can_ids.h"
#include "uwb/uwb_hal.h"
#include "nfc/rfid_hal.h"

Preferences carPreferences;
String g_carId;

static void LoadCarId()
{
    g_carId = carPreferences.getString("car_id", "");
}

static void SaveCarId(const String& carId)
{
    carPreferences.putString("car_id", carId);
    g_carId = carId;
}
constexpr uint32_t KEY_REQUEST_RETRY_DELAY_MS = 5000;

void PrintHex(const char* label, const uint8_t* data, size_t length);
constexpr uint32_t ISOTP_TIMEOUT_MS = 2000;
constexpr uint32_t NFC_DEBOUNCE_MS = 3000;

TaskHandle_t TaskBLE_Handle = nullptr;
TaskHandle_t TaskLogic_Handle = nullptr;
TaskHandle_t TaskKeyRequest_Handle = nullptr;
TaskHandle_t TaskNFC_Handle = nullptr;
TaskHandle_t TaskNFCProvisioning_Handle = nullptr;

static uint8_t savedNonce[16];
static bool waitingForResponse = false;
static uint32_t challengeSendTime = 0;
static volatile bool hasRealKey = false;
static SemaphoreHandle_t keyRequestTrigger;

/*=====================================================
    NFC WHITELIST (backup unlock khi khong co mang/BLE)
=====================================================
 */
constexpr uint8_t NFC_MAX_ENTRIES = 50;
constexpr uint8_t NFC_MAX_UID_LEN = 10;

struct NfcWhitelistEntry
{
    uint8_t uid[NFC_MAX_UID_LEN];
    uint8_t len;
};

Preferences nfcPreferences;
static NfcWhitelistEntry g_nfcWhitelist[NFC_MAX_ENTRIES];
static uint8_t g_nfcWhitelistCount = 0;

static bool HexCharToNibble(char c, uint8_t& outNibble)
{
    if (c >= '0' && c <= '9') { outNibble = c - '0'; return true; }
    if (c >= 'a' && c <= 'f') { outNibble = c - 'a' + 10; return true; }
    if (c >= 'A' && c <= 'F') { outNibble = c - 'A' + 10; return true; }
    return false;
}

static bool HexStringToUid(const String& hex, uint8_t* outUid, uint8_t maxLen, uint8_t& outLen)
{
    size_t hexLen = hex.length();
    if (hexLen == 0 || hexLen % 2 != 0)
        return false;

    size_t byteLen = hexLen / 2;
    if (byteLen > maxLen)
        return false;

    for (size_t i = 0; i < byteLen; i++)
    {
        uint8_t hi, lo;
        if (!HexCharToNibble(hex.charAt(i * 2), hi) ||
            !HexCharToNibble(hex.charAt(i * 2 + 1), lo))
            return false;

        outUid[i] = (uint8_t)((hi << 4) | lo);
    }

    outLen = (uint8_t)byteLen;
    return true;
}

static String UidToHexString(const uint8_t* uid, uint8_t len)
{
    String s;
    s.reserve(len * 2);
    for (uint8_t i = 0; i < len; i++)
    {
        if (uid[i] < 0x10) s += '0';
        s += String(uid[i], HEX);
    }
    s.toUpperCase();
    return s;
}

/*=====================================================
    AUDIT LOG (NVS) - ghi lại mỗi lần mở khóa bằng NFC
=====================================================
 */
constexpr uint8_t NFC_AUDIT_MAX_ENTRIES = 10;

static void AppendNfcAuditLog(const String& uidHex, uint32_t timestamp)
{
    String existing = nfcPreferences.getString("nfc_audit", "");

    // Đếm số entry hiện có, nếu đầy thì bỏ entry cũ nhất (đầu chuỗi).
    uint8_t count = 0;
    for (size_t i = 0; i < existing.length(); i++)
    {
        if (existing.charAt(i) == ',') count++;
    }
    if (existing.length() > 0) count++; // entry cuối không có dấu phẩy theo sau

    if (count >= NFC_AUDIT_MAX_ENTRIES)
    {
        int firstComma = existing.indexOf(',');
        if (firstComma != -1)
            existing = existing.substring(firstComma + 1);
    }

    String newEntry = uidHex + "@" + String(timestamp);
    if (existing.length() > 0)
        existing += "," + newEntry;
    else
        existing = newEntry;

    nfcPreferences.putString("nfc_audit", existing);
}

static void SaveNfcWhitelistToNVS()
{
    String combined;
    for (uint8_t i = 0; i < g_nfcWhitelistCount; i++)
    {
        if (i > 0) combined += ",";
        combined += UidToHexString(g_nfcWhitelist[i].uid, g_nfcWhitelist[i].len);
    }
    nfcPreferences.putString("nfc_whitelist", combined);
}

static void LoadNfcWhitelistFromNVS()
{
    g_nfcWhitelistCount = 0;
    String combined = nfcPreferences.getString("nfc_whitelist", "");
    if (combined.length() == 0)
        return;

    int start = 0;
    while (start < (int)combined.length() && g_nfcWhitelistCount < NFC_MAX_ENTRIES)
    {
        int comma = combined.indexOf(',', start);
        String token = (comma == -1) ? combined.substring(start) : combined.substring(start, comma);

        uint8_t uid[NFC_MAX_UID_LEN];
        uint8_t len;
        if (HexStringToUid(token, uid, NFC_MAX_UID_LEN, len))
        {
            memcpy(g_nfcWhitelist[g_nfcWhitelistCount].uid, uid, len);
            g_nfcWhitelist[g_nfcWhitelistCount].len = len;
            g_nfcWhitelistCount++;
        }

        if (comma == -1) break;
        start = comma + 1;
    }
}

static bool IsUidWhitelisted(const uint8_t* uid, uint8_t len)
{
    for (uint8_t i = 0; i < g_nfcWhitelistCount; i++)
    {
        if (g_nfcWhitelist[i].len == len &&
            memcmp(g_nfcWhitelist[i].uid, uid, len) == 0)
        {
            return true;
        }
    }
    return false;
}

/*=====================================================
    TÁC VỤ: NẠP/XOÁ UID QUA SERIAL (giống pattern Keyfob)
=====================================================
 * Lệnh:
 *   NFC_ADD:<hex_uid>   - thêm 1 UID vào whitelist
 *   NFC_DEL:<hex_uid>   - xoá 1 UID khỏi whitelist
 *   NFC_CLEAR           - xoá toàn bộ whitelist
 *   NFC_LIST             - in danh sách hiện tại
 *   NFC_AUDIT            - in audit log (lịch sử mở khóa qua NFC)
 * Phản hồi: "SUCCESS" hoặc "ERROR:<lý do>".
 */
static String ReadSerialLine()
{
    static String buffer;

    while (Serial.available())
    {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r')
        {
            if (buffer.length() == 0)
            {
                continue;
            }

            String line = buffer;
            buffer = "";
            return line;
        }

        buffer += c;
    }

    return ""; 
}

void TaskNFCProvisioning(void *pvParameters)
{
    for (;;)
    {
        if (Serial.available())
        {
            String line = ReadSerialLine();
            line.trim();

            if (line.length() == 0)
            {
                
            }
            else if (line.startsWith("NFC_ADD:"))
            {
                String hexUid = line.substring(8);
                uint8_t uid[NFC_MAX_UID_LEN];
                uint8_t len;

                if (!HexStringToUid(hexUid, uid, NFC_MAX_UID_LEN, len))
                {
                    LOG_PRINTLN("ERROR:UID khong dung dinh dang hex");
                }
                else if (g_nfcWhitelistCount >= NFC_MAX_ENTRIES)
                {
                    LOG_PRINTLN("ERROR:Whitelist da day");
                }
                else
                {
                    memcpy(g_nfcWhitelist[g_nfcWhitelistCount].uid, uid, len);
                    g_nfcWhitelist[g_nfcWhitelistCount].len = len;
                    g_nfcWhitelistCount++;
                    SaveNfcWhitelistToNVS();
                    LOG_PRINTLN("SUCCESS");
                }
            }
            else if (line.startsWith("NFC_DEL:"))
            {
                String hexUid = line.substring(8);
                uint8_t uid[NFC_MAX_UID_LEN];
                uint8_t len;

                if (!HexStringToUid(hexUid, uid, NFC_MAX_UID_LEN, len))
                {
                    LOG_PRINTLN("ERROR:UID khong dung dinh dang hex");
                }
                else
                {
                    bool found = false;
                    for (uint8_t i = 0; i < g_nfcWhitelistCount; i++)
                    {
                        if (g_nfcWhitelist[i].len == len &&
                            memcmp(g_nfcWhitelist[i].uid, uid, len) == 0)
                        {
                            // Dồn mảng để lấp chỗ trống thay vì để lỗ hổng.
                            for (uint8_t j = i; j < g_nfcWhitelistCount - 1; j++)
                            {
                                g_nfcWhitelist[j] = g_nfcWhitelist[j + 1];
                            }
                            g_nfcWhitelistCount--;
                            found = true;
                            break;
                        }
                    }

                    if (found)
                    {
                        SaveNfcWhitelistToNVS();
                        LOG_PRINTLN("SUCCESS");
                    }
                    else
                    {
                        LOG_PRINTLN("ERROR:UID khong ton tai trong whitelist");
                    }
                }
            }
            else if (line == "NFC_CLEAR")
            {
                g_nfcWhitelistCount = 0;
                SaveNfcWhitelistToNVS();
                LOG_PRINTLN("SUCCESS");
            }
            else if (line == "NFC_LIST")
            {
                LOG_PRINTF("Whitelist (%u thẻ):\n", g_nfcWhitelistCount);
                for (uint8_t i = 0; i < g_nfcWhitelistCount; i++)
                {
                    LOG_PRINTLN(("  " + UidToHexString(g_nfcWhitelist[i].uid, g_nfcWhitelist[i].len)).c_str());
                }
            }
            else if (line == "GET_CAR_ID")
            {
                LOG_PRINTF("CAR_ID:%s\n", g_carId.c_str());
            }
            else if (line.startsWith("SET_CAR_ID:"))
            {
                
                String newCarId = line.substring(11);
                newCarId.trim();

                if (newCarId.length() == 0)
                {
                    LOG_PRINTLN("ERROR:car_id rong");
                }
                else
                {
                    
                    bool carIdChanged = (g_carId.length() > 0) && (g_carId != newCarId);

                    SaveCarId(newCarId);

                    if (carIdChanged)
                    {
                        g_nfcWhitelistCount = 0;
                        SaveNfcWhitelistToNVS();
                        LOG_PRINTLN("[CAR PROVISION] car_id doi khac - da xoa sach whitelist NFC cu");
                    }

                    LOG_PRINTLN("SUCCESS");
                }
            }
            else if (line.startsWith("NFC_WRITE_BOOKING:"))
            {
                // Ghi booking_id (mã Firebase push key, 20 ký tự) vào
                // thẻ - ghép 3 block (48 byte), đủ chứa nguyên mã,
                // không cần rút gọn. CẦN THẺ ĐANG ÁP SÁT ĐẦU ĐỌC lúc
                // gọi lệnh này.
                String bookingId = line.substring(19);

                uint8_t uid[NFC_MAX_UID_LEN];
                uint8_t uidLen;

                if (!NFC_TryReadCard(uid, uidLen, NFC_MAX_UID_LEN))
                {
                    LOG_PRINTLN("ERROR:Khong thay the - ap the vao dau doc roi thu lai");
                }
                else if (bookingId.length() > NFC_DATA_MAX_LEN - 1)
                {
                    LOG_PRINTLN("ERROR:booking_id qua dai");
                    NFC_EndSession();
                }
                else
                {
                    if (NFC_WriteData((const uint8_t*)bookingId.c_str(), bookingId.length()))
                    {
                        LOG_PRINTLN("SUCCESS");
                    }
                    else
                    {
                        LOG_PRINTLN("ERROR:Ghi that bai (xem log NFC ERROR de biet ly do)");
                    }

                    NFC_EndSession();
                }
            }
            else if (line == "NFC_READ_BOOKING")
            {
                // Đọc lại booking_id đã ghi trên thẻ, in ra dạng text.
                uint8_t uid[NFC_MAX_UID_LEN];
                uint8_t uidLen;

                if (!NFC_TryReadCard(uid, uidLen, NFC_MAX_UID_LEN))
                {
                    LOG_PRINTLN("ERROR:Khong thay the - ap the vao dau doc roi thu lai");
                }
                else
                {
                    uint8_t data[NFC_DATA_MAX_LEN + 1] = {};
                    size_t dataLen = 0;

                    if (NFC_ReadData(data, NFC_DATA_MAX_LEN, dataLen))
                    {
                        data[dataLen] = '\0';
                        LOG_PRINTF("NFC_BOOKING_DATA:%s\n", (char*)data);
                    }
                    else
                    {
                        LOG_PRINTLN("ERROR:Doc that bai (xem log NFC ERROR de biet ly do)");
                    }

                    NFC_EndSession();
                }
            }
            else if (line == "NFC_AUDIT")
            {
                String log = nfcPreferences.getString("nfc_audit", "");
                LOG_PRINTLN("Audit log (backup unlock):");
                if (log.length() == 0)
                {
                    LOG_PRINTLN("  (chua co lan mo khoa nao qua NFC)");
                }
                else
                {
                    int start = 0;
                    while (start < (int)log.length())
                    {
                        int comma = log.indexOf(',', start);
                        String entry = (comma == -1) ? log.substring(start) : log.substring(start, comma);
                        LOG_PRINTLN(("  " + entry).c_str());
                        if (comma == -1) break;
                        start = comma + 1;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*=====================================================
    TÁC VỤ: ĐỌC THẺ NFC, SO SÁNH WHITELIST (backup unlock)
=====================================================
 * 3 cơ chế bảo vệ bổ sung (đúng ý "chỉ NFC không đủ an toàn"):
 *
 * 1. DEBOUNCE: thẻ còn trong vùng đọc không bị xử lý/log lặp lại
 *    trong NFC_DEBOUNCE_MS.
 *
 * 2. COOLDOWN SAU MỞ KHÓA: sau khi mở khóa thành công qua NFC, dừng
 *    hẳn việc quét/log trong NFC_UNLOCK_COOLDOWN_MS (~30s) - mô phỏng
 *    "đã mở rồi, không cần quét liên tục nữa cho tới khi khóa lại".
 *    Chưa có tín hiệu "khóa lại" thật (chưa có cảm biến cửa/relay) nên
 *    tạm dùng timeout làm điều kiện tự động "khóa lại".
 *
 * 3. RATE-LIMIT CHỐNG BRUTE-FORCE: nếu có quá nhiều thẻ KHÔNG hợp lệ
 *    quẹt liên tục trong thời gian ngắn (nghi ngờ đang dò/thử thẻ nhân
 *    bản), tạm khóa hẳn NFC trong NFC_LOCKOUT_MS - giống cơ chế
 *    cooldown đã áp dụng cho BLE trước đó.
 *
 * Mỗi lần mở khóa thành công qua NFC đều được ghi vào audit log (NVS)
 * để truy vết sau này.
 */
constexpr uint32_t NFC_UNLOCK_COOLDOWN_MS = 30000;
constexpr uint8_t NFC_MAX_FAILED_ATTEMPTS = 5;
constexpr uint32_t NFC_FAILED_WINDOW_MS = 60000;
constexpr uint32_t NFC_LOCKOUT_MS = 300000; // 5 phút

void TaskNFC(void *pvParameters)
{
    if (!NFC_Init())
    {
        LOG_PRINTLN("[CAR NFC ERROR] RC522 init failed");
        vTaskDelete(NULL);
    }

    LOG_PRINTLN("[CAR NFC] Sẵn sàng quét thẻ...");

    uint8_t lastUid[NFC_MAX_UID_LEN] = {};
    uint8_t lastUidLen = 0;
    uint32_t lastUidTime = 0;

    uint32_t unlockCooldownUntil = 0;
    uint32_t lockoutUntil = 0;
    uint8_t failedAttemptCount = 0;
    uint32_t failedWindowStart = 0;

    for (;;)
    {
        uint32_t now = millis();

        // Đang trong cooldown sau khi vừa mở khóa - bỏ qua hoàn toàn,
        // không đọc thẻ, không log.
        if (now < unlockCooldownUntil)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Đang bị lockout do brute-force - bỏ qua, chỉ nhắc 1 lần khi
        // vừa bắt đầu lockout (tránh log lặp suốt 5 phút).
        if (now < lockoutUntil)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint8_t uid[NFC_MAX_UID_LEN];
        uint8_t uidLen;

        if (NFC_TryReadCard(uid, uidLen, NFC_MAX_UID_LEN))
        {
            bool sameAsLast = (uidLen == lastUidLen) && (memcmp(uid, lastUid, uidLen) == 0);
            bool withinDebounce = (now - lastUidTime) < NFC_DEBOUNCE_MS;

            if (sameAsLast && withinDebounce)
            {
                // Thẻ vẫn đang trong vùng đọc, vừa xử lý gần đây - bỏ qua.
            }
            else
            {
                memcpy(lastUid, uid, uidLen);
                lastUidLen = uidLen;
                lastUidTime = now;

                String uidHex = UidToHexString(uid, uidLen);
                LOG_PRINTF("UID_DETECTED:%s\n", uidHex.c_str());

                if (IsUidWhitelisted(uid, uidLen))
                {
                    if (!hasRealKey)
                    {
                        LOG_PRINTF("[CAR NFC] Mã số thẻ %s hợp lệ. Mở khóa thành công\n",
                                   uidHex.c_str());
                        // TODO: gọi hàm mở khóa thật khi có (hiện chưa có
                        // logic "unlock" cụ thể ngoài log).

                        AppendNfcAuditLog(uidHex, now);

                        unlockCooldownUntil = now + NFC_UNLOCK_COOLDOWN_MS;
                        LOG_PRINTF("[CAR NFC] Đã mở khóa xe. Tạm dừng quét trong %lu ms\n",
                                   (unsigned long)NFC_UNLOCK_COOLDOWN_MS);

                        // Mở khóa thành công - reset bộ đếm rate-limit.
                        failedAttemptCount = 0;
                    }
                    else
                    {
                        LOG_PRINTF("[CAR NFC] Mã số thẻ %s hợp lệ nhưng BLE/UWB đã sẵn sàng. Từ chối NFC, cần xác thực qua BLE/UWB\n",
                                   uidHex.c_str());
                    }
                }
                else
                {
                    LOG_PRINTF("[CAR NFC] Mã số thẻ %s không hợp lệ, từ chối mở cửa.\n", uidHex.c_str());

                    // Rate-limit: đếm số lần thẻ sai trong cửa sổ thời
                    // gian gần nhất, nếu vượt ngưỡng thì lockout.
                    if (now - failedWindowStart > NFC_FAILED_WINDOW_MS)
                    {
                        failedWindowStart = now;
                        failedAttemptCount = 0;
                    }
                    failedAttemptCount++;

                    if (failedAttemptCount >= NFC_MAX_FAILED_ATTEMPTS)
                    {
                        lockoutUntil = now + NFC_LOCKOUT_MS;
                        LOG_PRINTF("[CAR NFC] CANH BAO: %u lần mã số thẻ sai trong %lu ms - khóa NFC %lu ms\n",
                                   failedAttemptCount,
                                   (unsigned long)NFC_FAILED_WINDOW_MS,
                                   (unsigned long)NFC_LOCKOUT_MS);
                        failedAttemptCount = 0;
                    }
                }
            }

            // BẮT BUỘC: kết thúc phiên làm việc với thẻ (Halt +
            // dừng Crypto1) - trước đây PICC_HaltA() tự động trong
            // NFC_TryReadCard(), giờ tách riêng để cho phép đọc/ghi
            // block dữ liệu (NFC_ReadBlock/NFC_WriteBlock) trước khi
            // đóng phiên. Nếu quên gọi, lần quẹt tiếp theo có thể lỗi.
            NFC_EndSession();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*=====================================================
        XIN KEY_ROOT TỪ GATEWAY QUA CAN (ISO-TP)
=====================================================*/
static bool RequestKeyRootFromGatewayOnce(bool verbose)
{
    KeyRequestPayload request = {};
    strncpy(request.car_id, g_carId.c_str(), sizeof(request.car_id) - 1);

    uint8_t reqBuf[KEY_REQUEST_PAYLOAD_SIZE];
    if (!SerializeKeyRequest(request, reqBuf, sizeof(reqBuf)))
    {
        LOG_PRINTLN("[CAR ERROR] Serialize KeyRequest that bai");
        return false;
    }

    if (!ISOTP_Send(CAN_ID_KEY_PROVISION_REQ, reqBuf, sizeof(reqBuf), ISOTP_TIMEOUT_MS))
    {
        if (verbose) LOG_PRINTLN("[CAR ERROR] Gửi yêu cầu xin key_root qua CAN thất bại");
        return false;
    }

    if (verbose) LOG_PRINTLN("[CAR] Đã gửi yêu cầu xin key_root qua CAN, chờ phản hồi từ Gateway...");

    uint8_t respBuf[KEY_RESPONSE_PAYLOAD_SIZE];
    size_t respLen = 0;
    if (!ISOTP_Receive(CAN_ID_KEY_PROVISION_RESP, respBuf, sizeof(respBuf), respLen, ISOTP_TIMEOUT_MS))
    {
        if (verbose) LOG_PRINTLN("[CAR ERROR] Không nhận được phản hồi từ Gateway");
        return false;
    }

    KeyResponsePayload response;
    if (!DeserializeKeyResponse(respBuf, respLen, response))
    {
        LOG_PRINTLN("[CAR ERROR] Deserialize KeyResponse that bai");
        return false;
    }

    Crypto_SetKey(response.key_root, response.key_len);
    LOG_PRINTF("[CAR] Đã nhận key_root (%u byte) từ Gateway qua CAN, đã áp dụng cho HMAC\n",
               (unsigned)response.key_len);
    PrintHex("[CAR DEBUG] Key_root nhận được:", response.key_root, response.key_len);
    return true;
}

/*=====================================================
        RETRY XIN KEY - EXPONENTIAL BACKOFF
=====================================================
 * Thay vì fix cứng 3s + ẩn bớt log (chỉ che triệu chứng), giãn thời
 * gian chờ tăng dần mỗi lần fail (3s -> 6s -> 12s -> 24s -> tối đa
 * 30s) - giảm tần suất gửi lên CAN bus THẬT SỰ, không chỉ giảm log.
 * Log vẫn in đầy đủ mỗi lần, nhưng tự nhiên thưa dần theo thời gian
 * thực do khoảng chờ giãn ra - không cần logic "verbose every N" giả
 * tạo nữa.
 */
void TaskKeyRequest(void *pvParameters)
{
    xSemaphoreTake(keyRequestTrigger, portMAX_DELAY);

    TWAI_Init();
    LOG_PRINTLN("[CAR] Keyfob kết nối lần đầu - bắt đầu xin key_root qua CAN...");

    for (;;)
    {
        if (RequestKeyRootFromGatewayOnce(true))
        {
            hasRealKey = true;
            LOG_PRINTLN("[CAR] key_root đã sẵn sàng, dừng retry.");
            vTaskDelete(NULL);
        }

        LOG_PRINTF("[CAR] Gateway chưa sẵn sàng, thử lại sau %lu ms...\n",
                   (unsigned long)KEY_REQUEST_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(KEY_REQUEST_RETRY_DELAY_MS));
    }
}

/*=====================================================
                    BLE Task
=====================================================*/
void PrintHex(const char* label, const uint8_t* data, size_t length) {
    LOG_PRINTF("%s ", label);
    for (size_t i = 0; i < length; i++) {
        LOG_PRINTF("%02X ", data[i]);
    }
    Serial.println();
}

void TaskBLE(void *pvParameters)
{
    if (!BLE_Car_Init())
    {
        LOG_PRINTLN("[BLE CAR] Init failed!");
        vTaskDelete(NULL);
    }

    for (;;)
    {
        BLE_Car_Task();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void SendChallenge()
{
    LOG_PRINTLN("[CAR AUTH ] Generating challenge");

    Packet txChallenge = {};
    txChallenge.type = PKT_CHALLENGE;
    txChallenge.length = 16;

    esp_fill_random(savedNonce, 16);
    memcpy(txChallenge.data, savedNonce, 16);
    PrintHex("[CAR DEBUG] Nonce :", savedNonce, 16);

    if (BLE_Car_SendPacket(txChallenge))
    {
        BLE_Car_SetState(CAR_CHALLENGE_SENT);
        waitingForResponse = true;
        challengeSendTime = millis();
    }
    else
    {
        LOG_PRINTLN("[CAR ERROR] Failed to send CHALLENGE");
    }
}

/*=====================================================
                Main Logic Task
=====================================================*/
void TaskLogic(void *pvParameters)
{
    Packet rxPacket;

    for (;;)
    {
        while (xQueueReceive(
                   bleRxQueue,
                   &rxPacket,
                   0) == pdTRUE)
        {
            switch (rxPacket.type)
            {
            case PKT_READY:
            {
                // So khớp car_id trước khi làm gì khác - đây CHỈ là bộ
                // lọc nhanh (tối ưu), KHÔNG phải lớp bảo mật chính.
                // Ngay cả khi Keyfob giả đúng car_id, HMAC verify vẫn
                // sẽ tự fail nếu key_root sai - bảo mật thật nằm ở đó.
                String receivedCarId((char*)rxPacket.data, rxPacket.length);

                if (rxPacket.length == 0 || receivedCarId != g_carId)
                {
                    LOG_PRINTF("[CAR] Keyfob báo cáo car_id = \"%s\" không khớp với xe này (%s) - từ chối kết nối\n",
                               receivedCarId.c_str(), g_carId.c_str());

                    Packet tx = {};
                    tx.type = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_CAR_ID_MISMATCH;
                    BLE_Car_SendPacket(tx);

                    // QUAN TRỌNG: đợi 1 chút trước khi disconnect - nếu
                    // ngắt ngay sau notify(), gói tin có thể chưa kịp
                    // truyền đi thực sự qua BLE, bị "rơi" giữa đường
                    // (đã xác nhận qua thực nghiệm - Keyfob không bao
                    // giờ nhận được AUTH_FAIL dù Car log đã gửi).
                    vTaskDelay(pdMS_TO_TICKS(200));
                    BLE_Car_Disconnect();
                    break;
                }

                if (hasRealKey)
                {
                    SendChallenge();
                }
                else
                {
                    xSemaphoreGive(keyRequestTrigger);
                    LOG_PRINTLN("[CAR] Keyfob đã kết nối, đang chờ key_root từ Gateway...");
                }
                break;
            }

            case PKT_RESPONSE:
            {
                waitingForResponse = false;

                LOG_PRINTLN("[CAR AUTH ] Verifying HMAC");

                if (rxPacket.length != 32)
                {
                    LOG_PRINTF("[CAR ERROR] Invalid RESPONSE length (%u)\n",
                                  rxPacket.length);

                    Packet tx = {};
                    tx.type = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_INVALID_LENGTH;
                    BLE_Car_SendPacket(tx);

                    // QUAN TRỌNG: đợi 1 chút trước khi disconnect - nếu
                    // ngắt ngay sau notify(), gói tin có thể chưa kịp
                    // truyền đi thực sự qua BLE, bị "rơi" giữa đường
                    // (đã xác nhận qua thực nghiệm - Keyfob không bao
                    // giờ nhận được AUTH_FAIL dù Car log đã gửi).
                    vTaskDelay(pdMS_TO_TICKS(200));
                    BLE_Car_Disconnect();
                    break;
                }

                PrintHex("[CAR DEBUG] Token :", rxPacket.data, 32);
                bool isValid = Crypto_Verify_HMAC(rxPacket.data, savedNonce);

                Packet tx = {};
                tx.length = 0;

                if (isValid)
                {
                    LOG_PRINTLN("[CAR AUTH ] SUCCESS");

                    BLE_Car_SetState(CAR_AUTHENTICATED);

                    tx.type = PKT_AUTH_OK;
                    if (!BLE_Car_SendPacket(tx))
                    {
                        LOG_PRINTLN("[CAR ERROR] Failed to send AUTH_OK");
                    }

                    if (UWB_Init() && UWB_StartRanging())
                    {
                        LOG_PRINTLN("[CAR UWB  ] Ranging started");
                    }
                    else
                    {
                        LOG_PRINTLN("[CAR ERROR] UWB start failed");
                    }
                }
                else
                {
                    LOG_PRINTLN("[CAR AUTH ] FAILED");

                    tx.type = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_HMAC_INVALID;
                    BLE_Car_SendPacket(tx);

                    // QUAN TRỌNG: đợi 1 chút trước khi disconnect - nếu
                    // ngắt ngay sau notify(), gói tin có thể chưa kịp
                    // truyền đi thực sự qua BLE, bị "rơi" giữa đường
                    // (đã xác nhận qua thực nghiệm - Keyfob không bao
                    // giờ nhận được AUTH_FAIL dù Car log đã gửi).
                    vTaskDelay(pdMS_TO_TICKS(200));
                    BLE_Car_Disconnect();
                }
                break;
            }

            default:
                LOG_PRINTF("[CAR ERROR] Unknown packet: 0x%02X\n", rxPacket.type);
                break;
            }
        }

        if (hasRealKey && BLE_Car_GetState() == CAR_KEY_RECEIVED)
        {
            SendChallenge();
        }

        if (waitingForResponse &&
            (millis() - challengeSendTime > BLE_TIMEOUT_MS))
        {
            LOG_PRINTLN("[CAR AUTH ] Response timeout");
            waitingForResponse = false;
            BLE_Car_Disconnect();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*=====================================================
                        Setup
=====================================================*/
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    LOG_PRINTLN("===============================");
    LOG_PRINTLN("      CAR ECU START");
    LOG_PRINTLN("===============================");

    if (!Queue_Init())
    {
        LOG_PRINTLN("[CAR ERROR] Queue init failed!");
        while (1) delay(1000);
    }

    carPreferences.begin("car_id_ns", false);
    LoadCarId();

    if (g_carId.length() == 0)
    {
        LOG_PRINTLN("[CAR PROVISION] Chưa có car_id - cho lệnh SET_CAR_ID:<value> qua Serial");
    }
    else
    {
        LOG_PRINTF("[CAR PROVISION] car_id đã có: %s\n", g_carId.c_str());
    }

    nfcPreferences.begin("car_nfc", false);
    LoadNfcWhitelistFromNVS();
    LOG_PRINTF("[CAR NFC] Đã nạp %u thẻ từ whitelist\n", g_nfcWhitelistCount);

    keyRequestTrigger = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(TaskBLE, "BLE", 4096, nullptr, 2, &TaskBLE_Handle, 0);
    xTaskCreatePinnedToCore(TaskLogic, "Logic", 4096, nullptr, 1, &TaskLogic_Handle, 1);
    xTaskCreatePinnedToCore(TaskKeyRequest, "KeyRequest", 4096, nullptr, 1, &TaskKeyRequest_Handle, 1);
    xTaskCreatePinnedToCore(TaskNFC, "NFC", 4096, nullptr, 1, &TaskNFC_Handle, 1);
    xTaskCreatePinnedToCore(TaskNFCProvisioning, "NFCProv", 4096, nullptr, 1, &TaskNFCProvisioning_Handle, 1);
}

void loop()
{
    vTaskDelete(NULL);
}