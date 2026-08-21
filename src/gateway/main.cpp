#include <Arduino.h>
#include "mbedtls/aes.h"

#include "can/twai_driver.h"
#include "can/isotp.h"
#include "can/payloads.h"
#include "can/can_ids.h"
#include "cloud/wifi_manager.h"
#include "cloud/firebase_manager.h"
#include "cloud/cloud_config.h"
#include "shared/config.h"

constexpr uint32_t ISOTP_TIMEOUT_MS = 2000;

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

static void HandleKeyRequest()
{
    uint8_t reqBuf[KEY_REQUEST_PAYLOAD_SIZE];
    size_t reqLen = 0;

    if (!ISOTP_Receive(CAN_ID_KEY_PROVISION_REQ, reqBuf, sizeof(reqBuf), reqLen, 100))
        return; // không có request nào tới, bình thường, thử lại vòng sau

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
        LOG_PRINTF("[GATEWAY] Đã giải mã và gửi key_root (%u byte) qua CAN cho Car có car_id=%s\n",
                   (unsigned)keyLen, request.car_id);
    }
    else
    {
        LOG_PRINTLN("[GATEWAY ERROR] Gửi KeyResponse qua CAN thất bại");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    LOG_PRINTLN("===============================");
    LOG_PRINTLN("   GATEWAY START");
    LOG_PRINTLN("===============================");

    TWAI_Init();

    if (!WiFi_Connect())
    {
        LOG_PRINTLN("[GATEWAY ERROR] WiFi connect failed");
        return;
    }

    if (!Firebase_Init())
    {
        LOG_PRINTLN("[GATEWAY ERROR] Firebase init failed");
        return;
    }

    LOG_PRINTLN("[GATEWAY] Sẵn sàng, chờ KeyRequest từ Car qua CAN...");
}

void loop()
{
    HandleKeyRequest();
}