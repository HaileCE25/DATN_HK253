#include <Arduino.h>

#include "keyfob/ble_key.h"
#include "os/queue.h"
#include "protocol/packet.h"
#include "crypto/hmac.h"

TaskHandle_t TaskBLE_Handle = nullptr;
TaskHandle_t TaskLogic_Handle = nullptr;

/*=====================================================
                    BLE Task
=====================================================*/
void PrintHex(const char* label, const uint8_t* data, size_t length) {
    Serial.printf("%s ", label);
    for(size_t i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void TaskBLE(void *pvParameters)
{
    // Serial.printf("[OS] BLE KEY Task running on Core %d\n",
    //               xPortGetCoreID());

    if (!BLE_Key_Init())
    {
        Serial.println("[KEY ERROR] BLE initialization failed");

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
    // Serial.printf("[OS] Logic KEY Task running on Core %d\n",
    //               xPortGetCoreID());

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
                    Serial.println("[KEY AUTH ] Challenge received");
                    Serial.println("[KEY AUTH ] Computing HMAC...");
                    PrintHex("[KEY DEBUG] Nonce :", rxPacket.data, 16);
                    Packet tx = {};
                    tx.type = PKT_RESPONSE;
                    
                    tx.length = 32; 

                    Crypto_Generate_HMAC(rxPacket.data, tx.data);
                    PrintHex("[KEY DEBUG] Token :", tx.data, 32);
                    if (BLE_Key_SendPacket(tx))
                    {
                        Serial.println("[KEY AUTH ] Waiting for authentication result...");
                    }
                    else
                    {
                        Serial.println("[KEY ERROR] Failed to send RESPONSE");
                    }
                    break;
                }
            case PKT_AUTH_OK:
                {
                    Serial.println("[KEY AUTH ] Authentication SUCCESS");

                    // TODO:
                    // Khởi động UWB Ranging
                    // Hoặc chuyển sang trạng thái chờ Unlock

                    break;
                }

                case PKT_AUTH_FAIL:
                {
                    Serial.println("[KEY AUTH ] Authentication FAILED");

                    // TODO:
                    // Reset session
                    // Quay về chờ Challenge mới

                    break;
                }

                // TODO: Bổ sung case PKT_AUTH_RESULT nếu xe gửi kết quả (Success/Fail) về
                // case PKT_AUTH_RESULT: 
                //     Kiểm tra kết quả, nếu thành công thì bật UWB

            default:
                Serial.printf("[KEY ERROR] Unknown packet: 0x%02X\n", rxPacket.type);
                break;
            }
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
    Serial.println("===============================");
    Serial.println("       KEYFOB START");
    Serial.println("===============================");

    if (!Queue_Init())
    {
        Serial.println("[OS] Queue init failed!");

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
}

void loop()
{
    vTaskDelete(NULL);
}