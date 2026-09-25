#include <Arduino.h>
#include <Preferences.h>
#include "protocol/packet.h"
#include "car/ble_car.h"
#include "os/queue.h"
#include "crypto/hmac.h"
#include "can/twai_driver.h"
#include "can/isotp.h"
#include "can/payloads.h"
#include "shared/vn_time.h"
#include "can/can_ids.h"
#include "uwb/uwb_hal.h"
#include "nfc/rfid_hal.h"
#include "lcd/lcd_display.h"

/*=====================================================
            TRẠNG THÁI FSM MỞ KHÓA
=====================================================*/
typedef enum
{
    FSM_IDLE,           // Chờ auth
    FSM_AUTH,           // Đang auth BLE (chưa xong)
    FSM_TRACKING,       // Auth OK, đang theo dõi khoảng cách UWB
    FSM_UNLOCK_WINDOW,  // Đã gần đủ lâu, chuẩn bị mở
    FSM_UNLOCKED,       // Đã mở khóa
    FSM_COOLDOWN,       // Vừa relock, tạm không cho mở lại ngay
} CarFsmState_t;

/*=====================================================
            NGƯỠNG UWB & FAIL-SAFE
=====================================================*/
constexpr float     UWB_R_UNLOCK      = 1.0f;    // m — vào gần hơn mức này → mở
constexpr float     UWB_R_LOCK        = 1.5f;    // m — ra xa hơn mức này → relock
constexpr uint32_t  UWB_T_STABLE_MS   = 3000;    // ms — phải ở trong R_UNLOCK liên tục
constexpr uint8_t   UWB_M_RELOCK      = 5;       // mẫu liên tiếp > R_LOCK → relock
constexpr uint8_t   UWB_M_UNLOCK_ABORT = 2;      // mẫu liên tiếp ra khỏi R_UNLOCK → mới reset dwell timer
                                                   // (chống dao động nhiễu quanh biên R_UNLOCK làm mất trắng bộ đếm)
constexpr uint32_t  BLE_FAILSAFE_MS   = 5000;    // ms — mất BLE lâu hơn → relock
constexpr uint32_t  FSM_COOLDOWN_MS   = 5000;    // ms — cooldown sau relock

/*=====================================================
            GLOBALS
=====================================================*/
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
// Gateway tra /Bookings + /SecureKeys (vài lời gọi HTTPS) trước khi trả lời.
constexpr uint32_t KEY_RESPONSE_TIMEOUT_MS = 5000;
// Đã có key vẫn hỏi lại Gateway theo chu kỳ này để phát hiện admin thu hồi
// (booking REVOKED). Thu hồi có hiệu lực chậm nhất sau khoảng này. Gateway
// không trả lời (mất mạng...) thì giữ key cũ - chỉ KEY_STATUS_REVOKED mới xoá.
constexpr uint32_t KEY_REVALIDATE_INTERVAL_MS = 10000;
// Bị thu hồi rồi thì Keyfob vẫn tự kết nối lại liên tục; trong khoảng này Car
// từ chối ngay, không hỏi Gateway (tránh mỗi lần connect lại là 1 lượt Firebase).
constexpr uint32_t KEY_REVOKED_RECHECK_MS = 30000;

void PrintHex(const char* label, const uint8_t* data, size_t length);
constexpr uint32_t ISOTP_TIMEOUT_MS = 2000;
constexpr uint32_t NFC_DEBOUNCE_MS = 3000;

TaskHandle_t TaskBLE_Handle          = nullptr;
TaskHandle_t TaskLogic_Handle        = nullptr;
TaskHandle_t TaskKeyRequest_Handle   = nullptr;
TaskHandle_t TaskNFC_Handle          = nullptr;
TaskHandle_t TaskNFCProvisioning_Handle = nullptr;
TaskHandle_t TaskUnlock_Handle       = nullptr;
TaskHandle_t TaskCarStatus_Handle    = nullptr;

static uint8_t savedNonce[16];
static bool waitingForResponse = false;
static uint32_t challengeSendTime = 0;
static volatile bool hasRealKey = false;
// Hạn của key_root theo Unix time UTC do Gateway gửi kèm (KEY_EXPIRE_UNKNOWN = chưa rõ).
// Hiện chỉ lưu và in log; chưa dùng để từ chối (Car chưa có nguồn thời gian thực).
static volatile uint32_t g_keyExpireUnix = KEY_EXPIRE_UNKNOWN;
static SemaphoreHandle_t keyRequestTrigger;

// Thu hồi key: TaskKeyRequest phát hiện, TaskLogic (chủ phiên BLE) huỷ phiên.
// g_keyRevoked chặn cả Keyfob lẫn thẻ NFC, lưu NVS để reset Car không mở lại được.
static volatile bool     g_keyRevokePending = false;
static volatile bool     g_keyRevoked       = false;
static volatile uint32_t g_keyRevokedAtMs   = 0;

static void SetKeyRevoked(bool revoked)
{
    if (revoked)
        g_keyRevokedAtMs = millis();

    if (g_keyRevoked == revoked)
        return; // chỉ ghi flash khi đổi trạng thái, không ghi mỗi lượt hỏi lại

    g_keyRevoked = revoked;
    carPreferences.putBool("key_revoked", revoked);
}

// K_session tính từ HKDF(key_root, nonce) sau khi auth thành công
static uint8_t  g_kSession[16];
static bool     g_kSessionReady = false;

// FSM và BLE fail-safe
static volatile CarFsmState_t g_fsmState  = FSM_IDLE;
static volatile uint32_t      g_lastBleMs = 0;  // millis() lần cuối có contact BLE

// Thời điểm auth bị từ chối gần nhất - chỉ để báo trạng thái lên gateway/LCD
// (sau khi fail car disconnect ngay, nếu không giữ lại thì LCD không kịp thấy).
static volatile bool     g_authFailSeen   = false;
static volatile uint32_t g_lastAuthFailMs = 0;

static void MarkAuthFail()
{
    g_lastAuthFailMs = millis();
    g_authFailSeen   = true;
}

/*=====================================================
    GỬI LỆNH ACTUATOR QUA CAN
=====================================================*/
static void SendActuatorCmd(uint8_t cmd)
{
    if (!ISOTP_Send(CAN_ID_ACTUATOR_CMD, &cmd, 1, ISOTP_TIMEOUT_MS))
    {
        LOG_PRINTF("[CAR ERROR] Gửi ActuatorCmd 0x%02X qua CAN thất bại\n", cmd);
        LCD_ShowMessage("CAN SEND FAIL", cmd == ACTUATOR_CMD_UNLOCK ? "UNLOCK cmd" : "LOCK cmd", 2000);
        return;
    }

    // Chỉ đổi LOCK/OPEN khi lệnh đã lên bus - không hiển thị trạng thái chưa thi hành
    LCD_SetLockState(cmd == ACTUATOR_CMD_UNLOCK);
}

/*=====================================================
    NFC WHITELIST (backup unlock khi không có mạng/BLE)
=====================================================*/
constexpr uint8_t NFC_MAX_ENTRIES  = 50;
constexpr uint8_t NFC_MAX_UID_LEN  = 10;

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
    if (hexLen == 0 || hexLen % 2 != 0) return false;
    size_t byteLen = hexLen / 2;
    if (byteLen > maxLen) return false;

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
    AUDIT LOG (NVS)
=====================================================*/
constexpr uint8_t NFC_AUDIT_MAX_ENTRIES = 10;

static void AppendNfcAuditLog(const String& uidHex, uint32_t timestamp)
{
    String existing = nfcPreferences.getString("nfc_audit", "");

    uint8_t count = 0;
    for (size_t i = 0; i < existing.length(); i++)
        if (existing.charAt(i) == ',') count++;
    if (existing.length() > 0) count++;

    if (count >= NFC_AUDIT_MAX_ENTRIES)
    {
        int firstComma = existing.indexOf(',');
        if (firstComma != -1) existing = existing.substring(firstComma + 1);
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
    if (combined.length() == 0) return;

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
            return true;
    }
    return false;
}

/*=====================================================
    TÁC VỤ: NẠP/XOÁ UID QUA SERIAL
=====================================================*/
static String ReadSerialLine()
{
    static String buffer;
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (buffer.length() == 0) continue;
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
                // bỏ qua dòng rỗng
            }
            else if (line.startsWith("NFC_ADD:"))
            {
                String hexUid = line.substring(8);
                uint8_t uid[NFC_MAX_UID_LEN];
                uint8_t len;

                if (!HexStringToUid(hexUid, uid, NFC_MAX_UID_LEN, len))
                    LOG_PRINTLN("ERROR:UID khong dung dinh dang hex");
                else if (g_nfcWhitelistCount >= NFC_MAX_ENTRIES)
                    LOG_PRINTLN("ERROR:Whitelist da day");
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
                            for (uint8_t j = i; j < g_nfcWhitelistCount - 1; j++)
                                g_nfcWhitelist[j] = g_nfcWhitelist[j + 1];
                            g_nfcWhitelistCount--;
                            found = true;
                            break;
                        }
                    }
                    if (found) { SaveNfcWhitelistToNVS(); LOG_PRINTLN("SUCCESS"); }
                    else LOG_PRINTLN("ERROR:UID khong ton tai trong whitelist");
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
                LOG_PRINTF("Whitelist (%u the):\n", g_nfcWhitelistCount);
                for (uint8_t i = 0; i < g_nfcWhitelistCount; i++)
                    LOG_PRINTLN(("  " + UidToHexString(g_nfcWhitelist[i].uid, g_nfcWhitelist[i].len)).c_str());
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
                        LOG_PRINTLN("SUCCESS");
                    else
                        LOG_PRINTLN("ERROR:Ghi that bai (xem log NFC ERROR de biet ly do)");
                    NFC_EndSession();
                }
            }
            else if (line == "NFC_READ_BOOKING")
            {
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
    TÁC VỤ: ĐỌC THẺ NFC (backup unlock)
=====================================================*/
constexpr uint32_t NFC_UNLOCK_COOLDOWN_MS  = 30000;
constexpr uint8_t  NFC_MAX_FAILED_ATTEMPTS = 5;
constexpr uint32_t NFC_FAILED_WINDOW_MS    = 60000;
constexpr uint32_t NFC_LOCKOUT_MS          = 300000; // 5 phút

void TaskNFC(void *pvParameters)
{
    if (!NFC_Init())
    {
        LOG_PRINTLN("[CAR NFC ERROR] RC522 init failed");
        vTaskDelete(NULL);
    }

    LOG_PRINTLN("[CAR NFC] San sang quet the...");

    uint8_t lastUid[NFC_MAX_UID_LEN] = {};
    uint8_t lastUidLen = 0;
    uint32_t lastUidTime = 0;

    uint32_t unlockCooldownUntil = 0;
    uint32_t lockoutUntil        = 0;
    uint8_t  failedAttemptCount  = 0;
    uint32_t failedWindowStart   = 0;

    for (;;)
    {
        uint32_t now = millis();

        if (now < unlockCooldownUntil) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        if (now < lockoutUntil)        { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        uint8_t uid[NFC_MAX_UID_LEN];
        uint8_t uidLen;

        if (NFC_TryReadCard(uid, uidLen, NFC_MAX_UID_LEN))
        {
            bool sameAsLast    = (uidLen == lastUidLen) && (memcmp(uid, lastUid, uidLen) == 0);
            bool withinDebounce = (now - lastUidTime) < NFC_DEBOUNCE_MS;

            if (!(sameAsLast && withinDebounce))
            {
                memcpy(lastUid, uid, uidLen);
                lastUidLen  = uidLen;
                lastUidTime = now;

                String uidHex = UidToHexString(uid, uidLen);
                LOG_PRINTF("UID_DETECTED:%s\n", uidHex.c_str());

                if (IsUidWhitelisted(uid, uidLen))
                {
                    // Booking bị thu hồi: thẻ NFC cũng mất quyền, như Keyfob.
                    if (g_keyRevoked)
                    {
                        LOG_PRINTF("[CAR NFC] The %s hop le nhung key da bi THU HOI - tu choi\n", uidHex.c_str());
                        LCD_ShowMessage("KEY REVOKED", "NFC denied", 2000);
                    }
                    // NFC chỉ là backup — chỉ cho phép khi không có BLE/UWB đang hoạt động.
                    else if (!hasRealKey ||
                        g_fsmState == FSM_IDLE ||
                        g_fsmState == FSM_COOLDOWN)
                    {
                        LOG_PRINTF("[CAR NFC] The %s hop le. Mo khoa (NFC backup)\n", uidHex.c_str());

                        // Gửi lệnh mở khóa thực qua CAN tới Gateway/Actuator
                        SendActuatorCmd(ACTUATOR_CMD_UNLOCK);

                        AppendNfcAuditLog(uidHex, now);
                        unlockCooldownUntil = now + NFC_UNLOCK_COOLDOWN_MS;
                        LOG_PRINTF("[CAR NFC] Da mo khoa xe. Tam dung quet trong %lu ms\n",
                                   (unsigned long)NFC_UNLOCK_COOLDOWN_MS);

                        failedAttemptCount = 0;
                    }
                    else
                    {
                        LOG_PRINTF("[CAR NFC] The %s hop le nhung BLE/UWB dang hoat dong (FSM=%d) - NFC bi bo qua\n",
                                   uidHex.c_str(), (int)g_fsmState);
                    }
                }
                else
                {
                    LOG_PRINTF("[CAR NFC] The %s khong hop le, tu choi mo cua.\n", uidHex.c_str());

                    if (now - failedWindowStart > NFC_FAILED_WINDOW_MS)
                    {
                        failedWindowStart  = now;
                        failedAttemptCount = 0;
                    }
                    failedAttemptCount++;

                    if (failedAttemptCount >= NFC_MAX_FAILED_ATTEMPTS)
                    {
                        lockoutUntil = now + NFC_LOCKOUT_MS;
                        LOG_PRINTF("[CAR NFC] CANH BAO: %u lan the sai - khoa NFC %lu ms\n",
                                   failedAttemptCount, (unsigned long)NFC_LOCKOUT_MS);
                        failedAttemptCount = 0;
                    }
                }
            }

            NFC_EndSession();
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/*=====================================================
    TÁC VỤ: UNLOCK FSM (theo dõi khoảng cách UWB)
=====================================================
 * FSM_TRACKING  → vào R_UNLOCK liên tục T_STABLE_MS → FSM_UNLOCKED
 * FSM_UNLOCKED  → ra ngoài R_LOCK M_RELOCK lần liên tiếp → FSM_COOLDOWN
 * BLE fail-safe → mất liên lạc BLE_FAILSAFE_MS → relock bất kể FSM
 */

// Gọi sau khi hết FSM_COOLDOWN. TRƯỚC ĐÂY luôn về FSM_IDLE, và ranging chỉ được
// bật lại khi Car nhận PKT_READY MỚI - nhưng Keyfob chỉ gửi PKT_READY DUY NHẤT
// mỗi lần BLE connect (xem case PKT_READY, ble_key.cpp), không gửi lại trên
// cùng 1 kết nối. Hệ quả: nếu BLE vẫn còn kết nối (rất thường gặp - tầm BLE xa
// hơn nhiều so với ngưỡng relock 1.5 m), Keyfob quay lại gần xe sau khi cooldown
// hết KHÔNG unlock lại được, phải rớt hẳn BLE rồi kết nối lại mới xin auth mới.
// Sửa: nếu BLE vẫn đang ở CAR_AUTHENTICATED (chưa hề rớt), bật lại ranging và
// vào thẳng FSM_TRACKING bằng K_session cũ - không cần challenge/response lại.
// (Lưu ý: Keyfob phía nó cũng không tự dừng ranging khi relock - xem TX task -
// nên khi Car bật lại RX, Keyfob đã "sẵn sàng" ở đó chờ ngay từ trước.)
static void ResumeTrackingOrIdleAfterCooldown()
{
    if (BLE_Car_GetState() == CAR_AUTHENTICATED && UWB_StartRanging())
    {
        LOG_PRINTLN("[CAR FSM ] Het cooldown, BLE con phien - resume ranging");
        g_fsmState = FSM_TRACKING;
    }
    else
    {
        g_fsmState = FSM_IDLE;
    }
}

void TaskUnlock(void *pvParameters)
{
    uint32_t inRangeStartMs   = 0;
    bool     inRangeStarted   = false;
    uint8_t  unlockAbortCount = 0; // mẫu liên tiếp ra khỏi R_UNLOCK (debounce dwell timer)
    uint8_t  outRangeCount    = 0;
    float    lastRelockDist      = 0.0f;
    bool     lastRelockDistValid = false; // để chỉ đếm outRangeCount trên mẫu MỚI, không đếm trùng mẫu cũ khi UWB duty-cycle (đo thưa sau unlock)

    for (;;)
    {
        // Chỉ chạy logic khi đang tracking
        CarFsmState_t state = g_fsmState;

        if (state == FSM_TRACKING || state == FSM_UNLOCK_WINDOW || state == FSM_UNLOCKED)
        {
            uint32_t now = millis();

            // ── BLE fail-safe ──────────────────────────────────────────────
            if ((now - g_lastBleMs) > BLE_FAILSAFE_MS)
            {
                LOG_PRINTLN("[CAR FSM ] BLE mat lien lac > fail-safe - relock");
                SendActuatorCmd(ACTUATOR_CMD_LOCK);
                UWB_StopRanging();
                g_fsmState          = FSM_COOLDOWN;
                inRangeStarted      = false;
                unlockAbortCount    = 0;
                outRangeCount       = 0;
                lastRelockDistValid = false;
                vTaskDelay(pdMS_TO_TICKS(FSM_COOLDOWN_MS));
                ResumeTrackingOrIdleAfterCooldown();
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // ── Đọc khoảng cách UWB ──────────────────────────────────────
            float dist;
            if (!UWB_GetLastDistance(dist))
            {
                // UWB chưa có mẫu - đợi thêm
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            // ── FSM_TRACKING / FSM_UNLOCK_WINDOW ─────────────────────────
            if (state == FSM_TRACKING || state == FSM_UNLOCK_WINDOW)
            {
                if (dist <= UWB_R_UNLOCK)
                {
                    unlockAbortCount = 0; // về lại trong vùng -> huỷ đếm "đang rời đi"

                    // Đang trong ngưỡng unlock
                    if (!inRangeStarted)
                    {
                        inRangeStarted = true;
                        inRangeStartMs = now;
                        g_fsmState = FSM_UNLOCK_WINDOW;
                        LOG_PRINTF("[CAR FSM ] Vao vung unlock (%.2f m), bat dau tinh gio...\n", dist);
                    }
                    // hasRealKey: key có thể vừa bị thu hồi giữa vòng lặp này (TaskLogic
                    // đang huỷ phiên) - không được mở khoá nữa.
                    else if ((now - inRangeStartMs) >= UWB_T_STABLE_MS && hasRealKey)
                    {
                        // Đủ thời gian → mở khóa
                        LOG_PRINTF("[CAR FSM ] Mo khoa! Giu %.2f m trong %lu ms\n",
                                   dist, (unsigned long)UWB_T_STABLE_MS);
                        SendActuatorCmd(ACTUATOR_CMD_UNLOCK);
                        g_fsmState    = FSM_UNLOCKED;
                        outRangeCount = 0;
                        inRangeStarted = false;
                        // Đo thưa CHỈ khi còn < UWB_R_UNLOCK (đứng sát xe); rời xa hơn để
                        // relock thì UWB tự quay lại full-rate + lọc bình thường (xem
                        // uwb_dw3000_rx.cpp::ShouldUseLowPowerNow()).
                        UWB_SetLowPowerMode(true, UWB_R_UNLOCK);
                    }
                }
                else
                {
                    // Ra ngoài ngưỡng — chỉ reset sau UWB_M_UNLOCK_ABORT mẫu liên tiếp,
                    // tránh 1 mẫu nhiễu dao động quanh biên R_UNLOCK xoá sạch dwell timer đã tích luỹ.
                    if (inRangeStarted)
                    {
                        unlockAbortCount++;
                        if (unlockAbortCount >= UWB_M_UNLOCK_ABORT)
                        {
                            LOG_PRINTF("[CAR FSM ] Ra khoi vung unlock (%.2f m) du %u mau lien tiep - reset timer\n",
                                       dist, (unsigned)unlockAbortCount);
                            inRangeStarted   = false;
                            unlockAbortCount = 0;
                            g_fsmState = FSM_TRACKING;
                        }
                    }
                }
            }
            // ── FSM_UNLOCKED ───────────────────────────────────────────────
            else if (state == FSM_UNLOCKED)
            {
                // UWB đang ở low-power mode (đo thưa mỗi ~2.5s) nhưng vòng lặp này vẫn
                // poll mỗi 100ms -> cùng 1 mẫu cũ sẽ được đọc lại nhiều lần. Chỉ đếm khi
                // giá trị THỰC SỰ đổi (mẫu mới), tránh outRangeCount tăng giả do đọc trùng.
                bool isNewSample = (!lastRelockDistValid) || (dist != lastRelockDist);
                lastRelockDist      = dist;
                lastRelockDistValid = true;

                if (dist > UWB_R_LOCK)
                {
                    if (isNewSample)
                    {
                        outRangeCount++;
                        LOG_PRINTF("[CAR FSM ] Xa vung lock (%.2f m), outRangeCount=%u/%u\n",
                                   dist, outRangeCount, UWB_M_RELOCK);
                    }

                    if (outRangeCount >= UWB_M_RELOCK)
                    {
                        LOG_PRINTLN("[CAR FSM ] Relock");
                        SendActuatorCmd(ACTUATOR_CMD_LOCK);
                        UWB_StopRanging();
                        g_fsmState    = FSM_COOLDOWN;
                        outRangeCount = 0;
                        lastRelockDistValid = false;
                        vTaskDelay(pdMS_TO_TICKS(FSM_COOLDOWN_MS));
                        ResumeTrackingOrIdleAfterCooldown();
                    }
                }
                else
                {
                    // Trở lại gần - reset bộ đếm
                    if (outRangeCount > 0 && isNewSample)
                    {
                        LOG_PRINTF("[CAR FSM ] Quay lai vung an toan (%.2f m), reset outRangeCount\n", dist);
                        outRangeCount = 0;
                    }
                }
            }
        }
        else
        {
            // FSM_IDLE / FSM_COOLDOWN - không cần làm gì, chờ
            inRangeStarted      = false;
            unlockAbortCount    = 0;
            outRangeCount       = 0;
            lastRelockDistValid = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*=====================================================
        XIN KEY_ROOT TỪ GATEWAY QUA CAN (ISO-TP)
=====================================================*/
enum KeyFetchResult
{
    KEY_FETCH_FAILED,   // không có/không hiểu phản hồi - không kết luận gì
    KEY_FETCH_OK,
    KEY_FETCH_REVOKED,  // Gateway xác nhận xe không còn booking ACTIVE
};

// verbose = false: lượt hỏi lại định kỳ - chỉ log khi có thay đổi.
static KeyFetchResult RequestKeyRootFromGatewayOnce(bool verbose)
{
    // TaskKeyRequest là nơi DUY NHẤT đọc CAN phía Car: bỏ các frame còn sót
    // (vd. phản hồi đến muộn của lượt trước) để không ráp nhầm vào lượt này.
    twai_message_t stale;
    while (TWAI_Receive(stale, 0)) {}

    KeyRequestPayload request = {};
    strncpy(request.car_id, g_carId.c_str(), sizeof(request.car_id) - 1);
    request.revalidate = !verbose;

    uint8_t reqBuf[KEY_REQUEST_PAYLOAD_SIZE];
    if (!SerializeKeyRequest(request, reqBuf, sizeof(reqBuf)))
    {
        LOG_PRINTLN("[CAR ERROR] Serialize KeyRequest that bai");
        return KEY_FETCH_FAILED;
    }

    if (!ISOTP_Send(CAN_ID_KEY_PROVISION_REQ, reqBuf, sizeof(reqBuf), ISOTP_TIMEOUT_MS))
    {
        if (verbose) LOG_PRINTLN("[CAR ERROR] Gui yeu cau xin Keyroot qua CAN that bai");
        return KEY_FETCH_FAILED;
    }

    if (verbose) LOG_PRINTLN("[CAR] Da gui yeu cau xin Keyroot qua CAN, cho phan hoi tu Gateway");

    uint8_t respBuf[KEY_RESPONSE_PAYLOAD_SIZE];
    size_t respLen = 0;
    if (!ISOTP_Receive(CAN_ID_KEY_PROVISION_RESP, respBuf, sizeof(respBuf), respLen, KEY_RESPONSE_TIMEOUT_MS))
    {
        if (verbose) LOG_PRINTLN("[CAR ERROR] Khong nhan duoc phan hoi tu Gateway");
        return KEY_FETCH_FAILED;
    }

    KeyResponsePayload response;
    if (!DeserializeKeyResponse(respBuf, respLen, response))
    {
        LOG_PRINTLN("[CAR ERROR] Deserialize KeyResponse that bai");
        return KEY_FETCH_FAILED;
    }

    if (response.status == KEY_STATUS_REVOKED)
    {
        LOG_PRINTLN("[CAR] Gateway bao key da bi THU HOI (khong con booking ACTIVE)");
        return KEY_FETCH_REVOKED;
    }

    bool expireChanged = response.expire_unix != g_keyExpireUnix;

    Crypto_SetKey(response.key_root, response.key_len);
    g_keyExpireUnix = response.expire_unix;

    if (verbose)
        LOG_PRINTF("[CAR] Da nhan Keyroot (%u byte) tu Gateway qua CAN\n", (unsigned)response.key_len);

    if (verbose || expireChanged)
    {
        if (response.expire_unix == KEY_EXPIRE_UNKNOWN)
            LOG_PRINTLN("[CAR] Khong nhan duoc thong tin het han");
        else
        {
            char expireVn[24];
            VnTime_Format(response.expire_unix, expireVn, sizeof(expireVn));
            LOG_PRINTF("[CAR] Key het han luc %s \n",
                       expireVn, (unsigned long)response.expire_unix);
        }
    }

    if (verbose)
    {
        LCD_ShowMessage("KEY_ROOT OK", "from Gateway", 2000);
        PrintHex("[CAR DEBUG] Keyroot nhan duoc:", response.key_root, response.key_len);
    }
    return KEY_FETCH_OK;
}

// Xoá key khỏi RAM và báo TaskLogic huỷ phiên đang chạy. hasRealKey = false
// trước tiên để TaskLogic không gửi CHALLENGE mới bằng key sắp bị xoá.
// Không fallback về key mặc định trong hmac.cpp: thiếu hasRealKey thì Car
// không bao giờ gửi CHALLENGE, nên key toàn 0 không dùng được để auth.
static void RevokeKey()
{
    hasRealKey = false;

    uint8_t zeroKey[32] = {};
    Crypto_SetKey(zeroKey, sizeof(zeroKey));
    g_keyExpireUnix = KEY_EXPIRE_UNKNOWN;

    SetKeyRevoked(true);
    g_keyRevokePending = true;

    LOG_PRINTLN("[CAR] Da xoa Keyroot - huy phien BLE/UWB, khoa ca the NFC");
}

// Exponential backoff: 5s → 10s → 20s → tối đa 30s. Có key rồi thì hỏi lại
// mỗi KEY_REVALIDATE_INTERVAL_MS; bị thu hồi thì quay lại chờ Keyfob kết nối.
// Lần đầu được kích ngay trong setup() (không đợi Keyfob) để Car biết trạng
// thái thu hồi sớm - thẻ NFC cần thông tin này dù không có Keyfob nào.
void TaskKeyRequest(void *pvParameters)
{
    for (;;)
    {
        xSemaphoreTake(keyRequestTrigger, portMAX_DELAY);

        LOG_PRINTLN("[CAR] Bat dau xin Keyroot qua CAN");

        uint32_t delayMs = KEY_REQUEST_RETRY_DELAY_MS;
        KeyFetchResult result;

        while ((result = RequestKeyRootFromGatewayOnce(true)) == KEY_FETCH_FAILED)
        {
            LOG_PRINTF("[CAR] Gateway chua san sang, thu lai sau %lu ms...\n", (unsigned long)delayMs);
            vTaskDelay(pdMS_TO_TICKS(delayMs));

            // Tăng gấp đôi, tối đa 30s
            delayMs = delayMs * 2;
            if (delayMs > 30000) delayMs = 30000;
        }

        if (result == KEY_FETCH_OK)
        {
            SetKeyRevoked(false);
            hasRealKey = true;
            LOG_PRINTLN("[CAR] Keyroot da san sang, chuyen sang kiem tra lai dinh ky.");

            do
            {
                vTaskDelay(pdMS_TO_TICKS(KEY_REVALIDATE_INTERVAL_MS));
            } while (RequestKeyRootFromGatewayOnce(false) != KEY_FETCH_REVOKED);
        }

        RevokeKey();

        // Bỏ trigger dồn lại trong lúc chờ Gateway - lần xin sau phải do
        // 1 lần Keyfob kết nối MỚI kích hoạt.
        xSemaphoreTake(keyRequestTrigger, 0);
    }
}

/*=====================================================
                    BLE Task
=====================================================*/
void PrintHex(const char* label, const uint8_t* data, size_t length)
{
    LOG_PRINTF("%s ", label);
    for (size_t i = 0; i < length; i++)
        LOG_PRINTF("%02X ", data[i]);
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
    txChallenge.type   = PKT_CHALLENGE;
    txChallenge.length = 16;

    esp_fill_random(savedNonce, 16);
    memcpy(txChallenge.data, savedNonce, 16);
    PrintHex("[CAR DEBUG] Nonce :", savedNonce, 16);

    if (BLE_Car_SendPacket(txChallenge))
    {
        BLE_Car_SetState(CAR_CHALLENGE_SENT);
        waitingForResponse = true;
        challengeSendTime  = millis();
        g_fsmState         = FSM_AUTH;
    }
    else
    {
        LOG_PRINTLN("[CAR ERROR] Failed to send CHALLENGE");
    }
}

static void RejectKeyfobRevoked()
{
    Packet tx = {};
    tx.type    = PKT_AUTH_FAIL;
    tx.length  = 1;
    tx.data[0] = AUTH_FAIL_REASON_KEY_REVOKED;
    BLE_Car_SendPacket(tx);
    MarkAuthFail();

    vTaskDelay(pdMS_TO_TICKS(200));
    BLE_Car_Disconnect();
}

// Huỷ phiên đang chạy sau khi key bị thu hồi (gọi từ TaskLogic). Đang mở khoá
// thì khoá lại ngay - không đợi Keyfob đi xa hay BLE fail-safe.
static void EndSessionOnKeyRevoked()
{
    CarFsmState_t state = g_fsmState;

    waitingForResponse = false;
    g_kSessionReady    = false;
    memset(g_kSession, 0, sizeof(g_kSession));

    if (state == FSM_TRACKING || state == FSM_UNLOCK_WINDOW || state == FSM_UNLOCKED)
        UWB_StopRanging();
    if (state == FSM_UNLOCKED)
        SendActuatorCmd(ACTUATOR_CMD_LOCK);
    if (state != FSM_COOLDOWN) // COOLDOWN: TaskUnlock tự đưa về IDLE khi hết (BLE đã ngắt)
        g_fsmState = FSM_IDLE;

    LCD_ShowMessage("KEY REVOKED", "Booking ended", 3000);

    if (BLE_Car_GetState() != CAR_IDLE)
        RejectKeyfobRevoked();
}

/*=====================================================
                Main Logic Task
=====================================================*/
void TaskLogic(void *pvParameters)
{
    Packet rxPacket;

    for (;;)
    {
        while (xQueueReceive(bleRxQueue, &rxPacket, 0) == pdTRUE)
        {
            switch (rxPacket.type)
            {
            case PKT_READY:
            {
                // Cập nhật thời điểm contact BLE gần nhất
                g_lastBleMs = millis();

                String receivedCarId((char*)rxPacket.data, rxPacket.length);

                if (rxPacket.length == 0 || receivedCarId != g_carId)
                {
                    LOG_PRINTF("[CAR] Keyfob car_id=\"%s\" khong khop xe nay (%s) - tu choi\n",
                               receivedCarId.c_str(), g_carId.c_str());

                    Packet tx = {};
                    tx.type   = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_CAR_ID_MISMATCH;
                    BLE_Car_SendPacket(tx);
                    MarkAuthFail();

                    vTaskDelay(pdMS_TO_TICKS(200));
                    BLE_Car_Disconnect();
                    break;
                }

                if (hasRealKey)
                {
                    SendChallenge();
                }
                else if (g_keyRevoked && (millis() - g_keyRevokedAtMs) < KEY_REVOKED_RECHECK_MS)
                {
                    LOG_PRINTLN("[CAR] Key vua bi thu hoi - tu choi Keyfob, chua hoi lai Gateway");
                    RejectKeyfobRevoked();
                }
                else
                {
                    xSemaphoreGive(keyRequestTrigger);
                    LOG_PRINTLN("[CAR] Keyfob da ket noi, dang cho key_root tu Gateway...");
                }
                break;
            }

            case PKT_RESPONSE:
            {
                waitingForResponse = false;
                g_lastBleMs = millis(); // liên lạc BLE OK

                LOG_PRINTLN("[CAR AUTH ] Verifying HMAC");

                if (rxPacket.length != 32)
                {
                    LOG_PRINTF("[CAR ERROR] Invalid RESPONSE length (%u)\n", rxPacket.length);

                    Packet tx = {};
                    tx.type   = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_INVALID_LENGTH;
                    BLE_Car_SendPacket(tx);
                    MarkAuthFail();

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
                        LOG_PRINTLN("[CAR ERROR] Failed to send AUTH_OK");

                    // ── M1: Tính K_session = HKDF(key_root, nonce) ──────────
                    // savedNonce là nonce đã dùng trong challenge vừa rồi.
                    g_kSessionReady = false;
                    if (!Crypto_HKDF(savedNonce, 16, g_kSession, 16))
                    {
                        LOG_PRINTLN("[CAR ERROR] HKDF that bai - ranging khong co session key");
                        // Vẫn init UWB với key mặc định (fallback)
                    }
                    else
                    {
                        g_kSessionReady = true;
                        LOG_PRINTLN("[CAR UWB  ] K_session computed via HKDF");
                        PrintHex("[CAR DEBUG] K_session:", g_kSession, 16);
                    }

                    // ── M2/M3: Init UWB + nạp K_session ────────────────────
                    if (!UWB_Init())
                    {
                        LOG_PRINTLN("[CAR ERROR] UWB Init that bai");
                        break;
                    }

                    if (g_kSessionReady)
                    {
                        if (!UWB_SetSessionKey(g_kSession, 16))
                            LOG_PRINTLN("[CAR ERROR] UWB_SetSessionKey that bai - ranging voi key mac dinh");
                        else
                            LOG_PRINTLN("[CAR UWB  ] K_session nap vao STS OK");
                    }

                    if (UWB_StartRanging())
                    {
                        LOG_PRINTLN("[CAR UWB  ] Ranging started");
                        g_fsmState  = FSM_TRACKING;
                        g_lastBleMs = millis();
                    }
                    else
                    {
                        LOG_PRINTLN("[CAR ERROR] UWB start failed");
                        g_fsmState = FSM_IDLE;
                    }
                }
                else
                {
                    LOG_PRINTLN("[CAR AUTH ] FAILED");
                    g_fsmState = FSM_IDLE;

                    tx.type   = PKT_AUTH_FAIL;
                    tx.length = 1;
                    tx.data[0] = AUTH_FAIL_REASON_HMAC_INVALID;
                    BLE_Car_SendPacket(tx);
                    MarkAuthFail();

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

        if (g_keyRevokePending)
        {
            g_keyRevokePending = false;
            EndSessionOnKeyRevoked();
        }

        // Cập nhật g_lastBleMs khi BLE đang ở trạng thái authenticated.
        // Nếu keyfob mất kết nối, state sẽ rời CAR_AUTHENTICATED → fail-safe bắt đầu đếm.
        if (BLE_Car_GetState() == CAR_AUTHENTICATED)
            g_lastBleMs = millis();

        if (hasRealKey && BLE_Car_GetState() == CAR_KEY_RECEIVED)
            SendChallenge();

        if (waitingForResponse &&
            (millis() - challengeSendTime > BLE_TIMEOUT_MS))
        {
            LOG_PRINTLN("[CAR AUTH ] Response timeout");
            MarkAuthFail();
            waitingForResponse = false;
            g_fsmState = FSM_IDLE;
            BLE_Car_Disconnect();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*=====================================================
    TÁC VỤ: PHÁT TRẠNG THÁI QUA CAN (hiển thị LCD gateway)
=====================================================
 * Gửi khi trạng thái đổi (tối đa mỗi CAR_STATUS_POLL_MS) + heartbeat mỗi
 * CAR_STATUS_HEARTBEAT_MS để gateway phát hiện car offline. Chỉ ĐỌC state
 * (UWB_GetLastDistance là non-blocking, không tiêu thụ mẫu của TaskUnlock).
 */
constexpr uint32_t CAR_STATUS_POLL_MS         = 100;
constexpr uint32_t CAR_STATUS_HEARTBEAT_MS    = 1000;
constexpr uint32_t CAR_STATUS_AUTH_FAIL_HOLD_MS = 3000;

static uint8_t MapBleStatus(uint32_t now)
{
    CarState bleState = BLE_Car_GetState();

    if (bleState == CAR_AUTHENTICATED)
        return CAR_STATUS_BLE_AUTHENTICATED;

    if (g_authFailSeen && (now - g_lastAuthFailMs) < CAR_STATUS_AUTH_FAIL_HOLD_MS)
        return CAR_STATUS_BLE_AUTH_FAIL;

    switch (bleState)
    {
    case CAR_CONNECTED:      return CAR_STATUS_BLE_CONNECTED;
    case CAR_KEY_RECEIVED:
    case CAR_CHALLENGE_SENT: return CAR_STATUS_BLE_AUTHENTICATING;
    case CAR_IDLE:
    default:                 return CAR_STATUS_BLE_ADVERTISING;
    }
}

static uint8_t MapFsmStatus(CarFsmState_t state)
{
    switch (state)
    {
    case FSM_AUTH:          return CAR_STATUS_FSM_AUTH;
    case FSM_TRACKING:      return CAR_STATUS_FSM_TRACKING;
    case FSM_UNLOCK_WINDOW: return CAR_STATUS_FSM_UNLOCK_WINDOW;
    case FSM_UNLOCKED:      return CAR_STATUS_FSM_UNLOCKED;
    case FSM_COOLDOWN:      return CAR_STATUS_FSM_COOLDOWN;
    case FSM_IDLE:
    default:                return CAR_STATUS_FSM_IDLE;
    }
}

static uint16_t DistanceToCm(float meters)
{
    if (meters <= 0.0f)
        return 0;

    float cm = meters * 100.0f + 0.5f;
    if (cm >= (float)(CAR_STATUS_DISTANCE_INVALID - 1))
        return CAR_STATUS_DISTANCE_INVALID - 1; // bão hoà, không đụng giá trị INVALID

    return (uint16_t)cm;
}

void TaskCarStatus(void *pvParameters)
{
    CarStatusPayload lastSent = {};
    bool     lastSendOk = false;
    uint32_t lastSentMs = 0;

    for (;;)
    {
        uint32_t now = millis();

        CarStatusPayload status = {};
        status.ble = MapBleStatus(now);
        status.fsm = MapFsmStatus(g_fsmState);

        float dist;
        status.distance_cm = UWB_GetLastDistance(dist) ? DistanceToCm(dist)
                                                       : CAR_STATUS_DISTANCE_INVALID;

        // LCD gắn tại car: cập nhật trực tiếp, không phụ thuộc CAN
        LCD_SetCarStatus(status);

        bool changed = status.ble != lastSent.ble ||
                       status.fsm != lastSent.fsm ||
                       status.distance_cm != lastSent.distance_cm;
        bool heartbeatDue = (now - lastSentMs) >= CAR_STATUS_HEARTBEAT_MS;

        // Lần gửi trước lỗi (gateway chưa bật, bus không ACK) -> chỉ thử lại theo
        // nhịp heartbeat, không spam log "[CAN ERROR] Transmit failed" mỗi 100ms.
        if ((changed && lastSendOk) || heartbeatDue)
        {
            uint8_t buf[CAR_STATUS_PAYLOAD_SIZE];
            lastSendOk = SerializeCarStatus(status, buf, sizeof(buf)) &&
                         ISOTP_Send(CAN_ID_CAR_STATUS, buf, sizeof(buf), 0);
            if (lastSendOk)
                lastSent = status;
            lastSentMs = now;
        }

        vTaskDelay(pdMS_TO_TICKS(CAR_STATUS_POLL_MS));
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

    // Không block: task LCD tự init I2C trên core 0, không có LCD vẫn chạy tiếp
    LCD_Init();

    if (!Queue_Init())
    {
        LOG_PRINTLN("[CAR ERROR] Queue init failed!");
        while (1) delay(1000);
    }

    carPreferences.begin("car_id_ns", false);
    LoadCarId();

    if (g_carId.length() == 0)
        LOG_PRINTLN("[CAR PROVISION] Chua co car_id - cho lenh SET_CAR_ID:<value> qua Serial");
    else
        LOG_PRINTF("[CAR PROVISION] car_id da co: %s\n", g_carId.c_str());

    nfcPreferences.begin("car_nfc", false);
    LoadNfcWhitelistFromNVS();
    LOG_PRINTF("[CAR NFC] Da nap %u the tu whitelist\n", g_nfcWhitelistCount);

    // TWAI khởi tạo ngay trong setup() — không lazy trong TaskKeyRequest nữa.
    TWAI_Init();

    keyRequestTrigger = xSemaphoreCreateBinary();

    g_keyRevoked = carPreferences.getBool("key_revoked", false);
    if (g_keyRevoked)
        LOG_PRINTLN("[CAR] Key dang o trang thai THU HOI (NVS) - Keyfob va the NFC bi chan");

    // Xin key ngay khi boot (cần car_id): cập nhật trạng thái thu hồi cho NFC
    // và bắt đầu vòng kiểm tra lại định kỳ mà không phải đợi Keyfob kết nối.
    if (g_carId.length() > 0)
        xSemaphoreGive(keyRequestTrigger);

    xTaskCreatePinnedToCore(TaskBLE,             "BLE",       4096, nullptr, 2, &TaskBLE_Handle,             0);
    xTaskCreatePinnedToCore(TaskLogic,           "Logic",     4096, nullptr, 1, &TaskLogic_Handle,           1);
    xTaskCreatePinnedToCore(TaskKeyRequest,      "KeyReq",    4096, nullptr, 1, &TaskKeyRequest_Handle,      1);
    // Core 0, không phải core 1: RangingTask UWB (priority 2, core 1) busy-wait
    // trong cửa sổ DS-TWR (xem uwb_dw3000_rx.cpp) sẽ chiếm hết CPU core 1, khiến
    // task priority 1 cùng core (kể cả NFC) gần như không được cấp CPU. Cùng lý
    // do LCD task đã dùng core 0 (xem lcd_config.h). NFC dùng bus SPI mặc định
    // (FSPI/SPI2) riêng với UWB (HSPI/SPI3) nên không tranh chấp phần cứng SPI.
    xTaskCreatePinnedToCore(TaskNFC,             "NFC",       4096, nullptr, 1, &TaskNFC_Handle,             0);
    xTaskCreatePinnedToCore(TaskNFCProvisioning, "NFCProv",   4096, nullptr, 1, &TaskNFCProvisioning_Handle, 1);
    xTaskCreatePinnedToCore(TaskUnlock,          "Unlock",    4096, nullptr, 1, &TaskUnlock_Handle,          1);
    xTaskCreatePinnedToCore(TaskCarStatus,       "CarStatus", 3072, nullptr, 1, &TaskCarStatus_Handle,       1);
}

void loop()
{
    vTaskDelete(NULL);
}
