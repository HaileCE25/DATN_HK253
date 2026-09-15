#include <Arduino.h>
#include <WiFi.h>
#include "mbedtls/aes.h"

#include "can/twai_driver.h"
#include "can/isotp.h"
#include "can/payloads.h"
#include "can/can_ids.h"
#include "cloud/wifi_manager.h"
#include "cloud/firebase_manager.h"
#include "cloud/cloud_config.h"
#include "shared/config.h"
#include "lcd/lcd_display.h"

constexpr uint32_t ISOTP_TIMEOUT_MS = 2000;
constexpr uint32_t CAN_DISPATCH_POLL_MS = 100;

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

    LOG_PRINTF("[GATEWAY] Yêu cầu xin key_root nhận được qua CAN, car_id=%s\n", request.car_id);

    String basePath = String("/SecureKeys/") + request.car_id;
    String ivHex, encryptedHex;

    if (!Firebase_ReadString((basePath + "/iv").c_str(), ivHex) ||
        !Firebase_ReadString((basePath + "/encrypted_key_root").c_str(), encryptedHex))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Firebase lookup thất bại (iv/encrypted_key_root)");
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
        LOG_PRINTLN("[GATEWAY ERROR] encrypted_key_root không đúng định dạng hex");
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
        LOG_PRINTLN("[GATEWAY ERROR] Sau khi giải mã, key_root không đúng định dạng hex");
        return;
    }

    response.key_len = (uint8_t)keyLen;
    PrintHex("[GATEWAY DEBUG] Key_root sau khi giải mã:", response.key_root, response.key_len);

    uint8_t respBuf[KEY_RESPONSE_PAYLOAD_SIZE];
    if (!SerializeKeyResponse(response, respBuf, sizeof(respBuf)))
    {
        LOG_PRINTLN("[GATEWAY ERROR] Serialize KeyResponse thất bại");
        return;
    }

    if (ISOTP_Send(CAN_ID_KEY_PROVISION_RESP, respBuf, sizeof(respBuf), ISOTP_TIMEOUT_MS))
    {
        LOG_PRINTF("[GATEWAY] Đã giải mã và gửi key_root (%u byte) qua CAN cho car_id=%s\n",
                   (unsigned)keyLen, request.car_id);
        LCD_ShowMessage("KEY_ROOT -> CAR", request.car_id, 2000);
    }
    else
    {
        LOG_PRINTLN("[GATEWAY ERROR] Gửi KeyResponse qua CAN thất bại");
    }
}

// =============================================================================
//  XỬ LÝ CAN_ID_ACTUATOR_CMD — nhận lệnh mở/khoá từ ECU Access, kích relay
// =============================================================================
static void HandleActuatorCommand(const twai_message_t& firstFrame)
{
    uint8_t cmdBuf[1];
    size_t cmdLen = 0;

    // Payload 1 byte luôn là Single Frame -> không cần chờ thêm frame nào
    if (!ISOTP_ReceiveFromFirstFrame(firstFrame, cmdBuf, sizeof(cmdBuf), cmdLen, 0))
    {
        LOG_PRINTLN("[GATEWAY ERROR] ActuatorCmd: frame khong hop le");
        return;
    }

    if (cmdLen < 1)
    {
        LOG_PRINTLN("[GATEWAY ERROR] ActuatorCmd: payload rỗng");
        return;
    }

    switch (cmdBuf[0])
    {
    case ACTUATOR_CMD_UNLOCK:
        LOG_PRINTLN("[GATEWAY ACTUATOR] UNLOCK - kich relay mo khoa");
        // TODO: digitalWrite(RELAY_PIN, HIGH) hoặc tín hiệu tương đương
        LCD_SetLockState(true);
        break;

    case ACTUATOR_CMD_LOCK:
        LOG_PRINTLN("[GATEWAY ACTUATOR] LOCK - tat relay, khoa lai");
        // TODO: digitalWrite(RELAY_PIN, LOW)
        LCD_SetLockState(false);
        break;

    default:
        LOG_PRINTF("[GATEWAY ERROR] ActuatorCmd unknown: 0x%02X\n", cmdBuf[0]);
        break;
    }
}

// =============================================================================
//  XỬ LÝ CAN_ID_CAR_STATUS — trạng thái BLE/FSM/UWB từ Car, chỉ để hiển thị
// =============================================================================
static void HandleCarStatus(const twai_message_t& firstFrame)
{
    uint8_t buf[CAR_STATUS_PAYLOAD_SIZE];
    size_t len = 0;

    if (!ISOTP_ReceiveFromFirstFrame(firstFrame, buf, sizeof(buf), len, 0))
        return;

    CarStatusPayload status;
    if (!DeserializeCarStatus(buf, len, status))
    {
        LOG_PRINTLN("[GATEWAY ERROR] CarStatus payload khong hop le");
        return;
    }

    LCD_SetCarStatus(status);
}

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

    // LCD init trước để hiển thị được tiến trình WiFi/Firebase (blocking vài giây).
    // Không có LCD thì gateway vẫn chạy bình thường.
    if (!LCD_Init())
        LOG_PRINTLN("[GATEWAY WARN] LCD khong san sang - chay tiep khong hien thi");

    s_canReady = TWAI_Init();

    // Các thông báo khởi động dưới đây có thời lượng dài hơn thời gian block của
    // bước tương ứng -> luôn hiện suốt lúc chờ, bước sau ghi đè bước trước.
    LCD_SetWifiState(LCD_LINK_PENDING);
    LCD_ShowMessage("GATEWAY START", "WiFi: " WIFI_SSID, WIFI_CONNECT_TIMEOUT_MS + 1000);
    if (!WiFi_Connect())
    {
        LOG_PRINTLN("[GATEWAY ERROR] WiFi connect failed");
        LCD_SetWifiState(LCD_LINK_FAIL);
        LCD_ShowMessage("WiFi FAIL", WIFI_SSID, 5000);
        return;
    }
    LCD_SetWifiState(LCD_LINK_OK);

    LCD_SetFirebaseState(LCD_LINK_PENDING);
    LCD_ShowMessage("WiFi OK", WiFi.localIP().toString().c_str(), 30000);
    if (!Firebase_Init())
    {
        LOG_PRINTLN("[GATEWAY ERROR] Firebase init failed");
        LCD_SetFirebaseState(LCD_LINK_FAIL);
        LCD_ShowMessage("Firebase FAIL", WiFi.localIP().toString().c_str(), 5000);
        return;
    }
    LCD_SetFirebaseState(LCD_LINK_OK);
    LCD_ShowMessage("Firebase OK", WiFi.localIP().toString().c_str(), 3000);

    LOG_PRINTLN("[GATEWAY] Sẵn sàng, chờ KeyRequest + ActuatorCmd + CarStatus từ Car qua CAN...");
}

// Vòng dispatch CAN DUY NHẤT: đọc 1 frame rồi route theo identifier.
// Không để từng handler tự đọc bus - ISOTP_Receive(id) vứt bỏ mọi frame khác
// ID trong lúc chờ, nên handler này sẽ "ăn mất" frame của handler kia.
void loop()
{
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
    case CAN_ID_ACTUATOR_CMD:      HandleActuatorCommand(frame); break;
    case CAN_ID_CAR_STATUS:        HandleCarStatus(frame);       break;
    default:                                                     break;
    }
}
