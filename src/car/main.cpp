#include <Arduino.h>
#include "protocol/packet.h" 
#include "car/ble_car.h"
#include "os/queue.h"
#include "crypto/hmac.h"

TaskHandle_t TaskBLE_Handle = nullptr;
TaskHandle_t TaskLogic_Handle = nullptr;

static uint8_t saved_nonce[16];
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
    //Serial.printf("[OS] BLE CAR Task running on Core %d\n", xPortGetCoreID());

    if (!BLE_Car_Init())
    {
        Serial.println("[BLE CAR] Init failed!");
        vTaskDelete(NULL);
    }

    for (;;)
    {
        BLE_Car_Task(); 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/*=====================================================
                Main Logic Task
=====================================================*/
void TaskLogic(void *pvParameters)
{
    //Serial.printf("[OS] Logic CAR Task running on Core %d\n",
    //              xPortGetCoreID());

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
            case PKT_READY: // BẮT BUỘC PHẢI CÓ CASE NÀY

                //Serial.println("[CAR RX   ] READY");
                Serial.println("[CAR AUTH ] Generating challenge"); 

                Packet txChallenge;
                txChallenge.type = PKT_CHALLENGE;
                txChallenge.length = 16;
                
                esp_fill_random(saved_nonce, 16);
                memcpy(txChallenge.data, saved_nonce, 16);
                PrintHex("[CAR DEBUG] Nonce :", saved_nonce, 16);
                if (!BLE_Car_SendPacket(txChallenge))
                {
                    Serial.println("[CAR ERROR] Failed to send CHALLENGE");
                }
                break;

            case PKT_RESPONSE:
            {
                //Serial.println("[CAR RX   ] RESPONSE");
                Serial.println("[CAR AUTH ] Verifying HMAC");
                PrintHex("[CAR DEBUG] Token :", rxPacket.data, 32);
                bool isValid = Crypto_Verify_HMAC(rxPacket.data, saved_nonce);
                Packet tx = {};
                tx.length = 0;
                if (isValid) 
                {
                    Serial.println("[CAR AUTH ] SUCCESS");
                    tx.type = PKT_AUTH_OK;

                    if (!BLE_Car_SendPacket(tx))
                    {
                        Serial.println("[CAR ERROR] Failed to send AUTH_OK");
                    }
                    
                    // TODO Bước tiếp theo: 
                    // Dẫn xuất K_session (HKDF)
                    // Bật chip UWB DW3000 sang chế độ Ranging
                    
                } 
                else 
                {
                    Serial.println("[CAR AUTH ] FAILED");
                    tx.type = PKT_AUTH_FAIL;

                    if (!BLE_Car_SendPacket(tx))
                    {
                        Serial.println("[CAR ERROR] AUTH_FAIL send failed");
                    }
                }
                break;
            }

            default:
                Serial.printf("[CAR ERROR] Unknown packet: 0x%02X\n", rxPacket.type);
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
    Serial.println("      CAR ECU START");
    Serial.println("===============================");

    if (!Queue_Init())
    {
        Serial.println("[CAR ERROR] Queue init failed!");
        while (1) delay(1000);
    }

    xTaskCreatePinnedToCore(TaskBLE, "BLE", 4096, nullptr, 2, &TaskBLE_Handle, 0);
    xTaskCreatePinnedToCore(TaskLogic, "Logic", 4096, nullptr, 1, &TaskLogic_Handle, 1);
}

void loop()
{
    vTaskDelete(NULL);
}