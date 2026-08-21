#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#include "keyfob/ble_key.h"
#include "os/queue.h"
#include "protocol/packet.h"
#include "crypto/hmac.h"

TaskHandle_t TaskBLE_Handle = nullptr;
TaskHandle_t TaskLogic_Handle = nullptr;
TaskHandle_t TaskProvisioning_Handle = nullptr;

static bool waitingForAuthResult = false;
static uint32_t responseSendTime = 0;

/*=====================================================
            Provisioning data (NVS)
=====================================================*/
Preferences preferences;

String g_carId;
String g_licensePlate;
String g_carModel;
String g_keyRoot;

static bool IsProvisioned()
{
    return g_keyRoot.length() > 0;
}

static void SaveProvisioningData()
{
    preferences.putString("car_id", g_carId);
    preferences.putString("license_plate", g_licensePlate);
    preferences.putString("car_model", g_carModel);
    preferences.putString("key_root", g_keyRoot);
}

static void LoadProvisioningData()
{
    g_carId = preferences.getString("car_id", "");
    g_licensePlate = preferences.getString("license_plate", "");
    g_carModel = preferences.getString("car_model", "");
    g_keyRoot = preferences.getString("key_root", "");
}

// Parse chuỗi hex (g_keyRoot) thành byte thô rồi nạp vào module crypto.
// THIẾU BƯỚC NÀY LÀ NGUYÊN NHÂN GÂY AUTH FAIL: trước đây key_root chỉ
// được đọc và in ra log, không hề được áp dụng - Crypto_Generate_HMAC
// vẫn dùng key mặc định hardcode trong hmac.cpp, khác với key_root
// thật, khiến verify luôn fail.
static bool ApplyKeyRootToCrypto()
{
    if (!IsProvisioned())
        return false;

    size_t hexLen = g_keyRoot.length();
    if (hexLen % 2 != 0)
    {
        LOG_PRINTLN("[KEY ERROR] key_root co do dai hex khong hop le");
        return false;
    }

    size_t byteLen = hexLen / 2;
    uint8_t keyBytes[32];
    if (byteLen > sizeof(keyBytes))
    {
        LOG_PRINTLN("[KEY ERROR] key_root qua dai");
        return false;
    }

    for (size_t i = 0; i < byteLen; i++)
    {
        char high = g_keyRoot.charAt(i * 2);
        char low = g_keyRoot.charAt(i * 2 + 1);

        auto hexDigit = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };

        int h = hexDigit(high);
        int l = hexDigit(low);
        if (h < 0 || l < 0)
        {
            LOG_PRINTLN("[KEY ERROR] key_root chua ky tu khong phai hex");
            return false;
        }

        keyBytes[i] = (uint8_t)((h << 4) | l);
    }

    Crypto_SetKey(keyBytes, byteLen);
    LOG_PRINTF("[KEY] Da ap dung key_root (%u byte) cho HMAC\n", (unsigned)byteLen);
    return true;
}

/*=====================================================
        TÁC VỤ: LẮNG NGHE SERIAL ĐỂ PROVISIONING
=====================================================*/

// Đọc 1 dòng từ Serial, chấp nhận CẢ \r, \n, VÀ \r\n - xem ghi chú chi
// tiết trong car/main.cpp (cùng bug, cùng cách sửa).
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

void TaskProvisioning(void *pvParameters)
{
    for (;;)
    {
        if (Serial.available())
        {
            String incomingData = ReadSerialLine();
            incomingData.trim();

            if (incomingData.length() == 0)
            {
                // Chưa đủ 1 dòng hoàn chỉnh - bỏ qua.
            }
            else if (incomingData.startsWith("PROVISION:"))
            {
                String jsonPayload = incomingData.substring(10);

                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, jsonPayload);

                if (err)
                {
                    LOG_PRINTF("ERROR:JSON parse failed - %s\n", err.c_str());
                }
                else if (!doc["car_id"].is<const char*>() ||
                         !doc["key_root"].is<const char*>())
                {
                    LOG_PRINTLN("ERROR:Missing required field (car_id or key_root)");
                }
                else
                {
                    String newCarId = doc["car_id"].as<String>();
                    String newKeyRoot = doc["key_root"].as<String>();

                    if (newCarId.length() == 0 || newKeyRoot.length() == 0)
                    {
                        LOG_PRINTLN("ERROR:car_id or key_root empty");
                    }
                    else
                    {
                        g_carId = newCarId;
                        g_keyRoot = newKeyRoot;
                        //g_licensePlate = doc["license_plate"] | "";
                        //g_carModel = doc["car_model"] | "";

                        SaveProvisioningData();

                        LOG_PRINTLN("SUCCESS");

                        vTaskDelay(pdMS_TO_TICKS(1000));
                        ESP.restart(); // Áp dụng dữ liệu mới - setup() sẽ
                                       // load lại + ApplyKeyRootToCrypto()
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*=====================================================
                    BLE Task
=====================================================*/
void PrintHex(const char* label, const uint8_t* data, size_t length) {
    LOG_PRINTF("%s ", label);
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void TaskBLE(void *pvParameters)
{
    if (!BLE_Key_Init())
    {
        LOG_PRINTLN("[KEY ERROR] BLE initialization failed");
        vTaskDelete(NULL);
    }

    for (;;)
    {
        BLE_Key_Task();
        vTaskDelay(pdMS_TO_TICKS(20));
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
            case PKT_CHALLENGE:
            {
                LOG_PRINTLN("[KEY AUTH ] Challenge received");

                if (rxPacket.length != 16)
                {
                    LOG_PRINTF("[KEY ERROR] Invalid CHALLENGE length (%u)\n",
                                  rxPacket.length);
                    break;
                }

                BLE_Key_SetState(KEY_CHALLENGE_RECEIVED);

                LOG_PRINTLN("[KEY AUTH ] Computing HMAC...");
                PrintHex("[KEY DEBUG] Nonce :", rxPacket.data, 16);

                Packet tx = {};
                tx.type = PKT_RESPONSE;
                tx.length = 32;

                Crypto_Generate_HMAC(rxPacket.data, tx.data);
                PrintHex("[KEY DEBUG] Token :", tx.data, 32);

                if (BLE_Key_SendPacket(tx))
                {
                    LOG_PRINTLN("[KEY AUTH ] Waiting for authentication result...");
                    waitingForAuthResult = true;
                    responseSendTime = millis();
                }
                else
                {
                    LOG_PRINTLN("[KEY ERROR] Failed to send RESPONSE");
                }
                break;
            }

            case PKT_AUTH_OK:
            {
                waitingForAuthResult = false;

                LOG_PRINTLN("[KEY AUTH ] Authentication SUCCESS");
                BLE_Key_SetState(KEY_AUTHENTICATED);
                break;
            }

            case PKT_AUTH_FAIL:
            {
                waitingForAuthResult = false;

                uint8_t reason = (rxPacket.length >= 1) ? rxPacket.data[0] : AUTH_FAIL_REASON_UNKNOWN;

                if (reason == AUTH_FAIL_REASON_CAR_ID_MISMATCH)
                {
                    // Lỗi VĨNH VIỄN - car_id đã sai từ gốc (không phải
                    // do thời điểm/key chưa đồng bộ). Dừng hẳn, không
                    // scan lại - tránh lãng phí pin/vô tuyến vô ích.
                    // Cần kiểm tra lại provisioning (car_id có đúng
                    // không) rồi mới nạp lại và restart board.
                    LOG_PRINTLN("[KEY AUTH ] THAT BAI VINH VIEN: car_id khong khop voi Car");
                    BLE_Key_HaltPermanently();
                }
                else
                {
                    // HMAC sai, độ dài gói bất thường, hoặc lý do khác
                    // có thể tự hết (vd Gateway vừa cập nhật key_root
                    // mới) - vẫn thử lại bình thường như trước.
                    LOG_PRINTLN("[KEY AUTH ] Authentication FAILED - se thu lai");
                    BLE_Key_Disconnect();
                }
                break;
            }

            default:
                LOG_PRINTF("[KEY ERROR] Unknown packet: 0x%02X\n", rxPacket.type);
                break;
            }
        }

        if (waitingForAuthResult &&
            (millis() - responseSendTime > BLE_TIMEOUT_MS))
        {
            LOG_PRINTLN("[KEY AUTH ] Auth result timeout");
            waitingForAuthResult = false;
            BLE_Key_Disconnect();
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
    LOG_PRINTLN("       KEYFOB START");
    LOG_PRINTLN("===============================");

    preferences.begin("smart_car", false);
    LoadProvisioningData();

    if (IsProvisioned())
    {
        LOG_PRINTLN("[PROVISION] Da co du lieu:");
        LOG_PRINTF("  car_id        : %s\n", g_carId.c_str());
        // LOG_PRINTF("  license_plate : %s\n", g_licensePlate.c_str());
        // LOG_PRINTF("  car_model     : %s\n", g_carModel.c_str());
        LOG_PRINTF("  key_root      : %s\n", g_keyRoot.c_str());

        if (!ApplyKeyRootToCrypto())
        {
            LOG_PRINTLN("[KEY ERROR] Ap dung key_root that bai - se dung key mac dinh (SE AUTH FAIL)");
        }

        // Gán car_id của xe này để gửi kèm trong gói READY - Car sẽ so
        // khớp trước khi tiếp tục (bộ lọc nhanh, không phải bảo mật
        // chính - HMAC vẫn là hàng rào thật).
        BLE_Key_SetOwnCarId(g_carId.c_str());
    }
    else
    {
        LOG_PRINTLN("[PROVISION] CHUA CO DU LIEU - cho Admin Tool cap phat qua Serial");
    }

    if (!Queue_Init())
    {
        LOG_PRINTLN("[OS] Queue init failed!");
        while (1)
        {
            delay(1000);
        }
    }

    xTaskCreatePinnedToCore(
        TaskBLE,
        "BLE",
        4096,
        nullptr,
        2,
        &TaskBLE_Handle,
        1);

    xTaskCreatePinnedToCore(
        TaskLogic,
        "Logic",
        4096,
        nullptr,
        1,
        &TaskLogic_Handle,
        1);

    xTaskCreatePinnedToCore(
        TaskProvisioning,
        "Provisioning",
        4096,
        nullptr,
        1,
        &TaskProvisioning_Handle,
        1);
}

void loop()
{
    vTaskDelete(NULL);
}