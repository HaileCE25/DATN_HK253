#include "keyfob/ble_key.h"

#include <NimBLEDevice.h>
#include "shared/config.h"
#include "os/queue.h"

/*=====================================================
                    INTERNAL STATE
=====================================================*/

static bool connected = false;
static bool doConnect = false;
static bool doScan = false;

static KeyState keyState = KEY_SCANNING;

static NimBLEAdvertisedDevice* pAdvertisedDevice = nullptr;
static NimBLEClient* pClient = nullptr;

static NimBLERemoteCharacteristic* pRemoteTx = nullptr;
static NimBLERemoteCharacteristic* pRemoteRx = nullptr;
/*=====================================================
                    NOTIFY CALLBACK
=====================================================*/

static void NotifyCallback(
    NimBLERemoteCharacteristic*,
    uint8_t* data,
    size_t length,
    bool)
{
    if (length < 2) return;

    uint8_t type = data[0];
    uint8_t len  = data[1];

    if (len > MAX_PACKET_SIZE) return;
    if (length < (size_t)(2 + len)) return;

    Packet pkt = {};
    pkt.type = (PacketType)type;
    pkt.length = len;

    memcpy(pkt.data, data + 2, len);

    /*================ STATE UPDATE ================*/
    Serial.printf("[KEY RX  ] %s\n",
              PacketTypeToString(pkt.type));

    switch (pkt.type)
    {
    case PKT_CHALLENGE:

        keyState = KEY_CHALLENGE_RECEIVED;
        break;

    case PKT_AUTH_OK:

        keyState = KEY_AUTHENTICATED;
        //Serial.println("[KEY AUTH] Authentication SUCCESS");
        break;

    case PKT_AUTH_FAIL:

        keyState = KEY_READY;
        //Serial.println("[KEY AUTH] Authentication FAILED");
        break;

    default:
        break;
    }

    if (xQueueSend(bleRxQueue, &pkt, 0) != pdPASS)
    {
        Serial.println("[KEY ERROR] RX queue full");
    }
}

/*=====================================================
                    CLIENT CALLBACK
=====================================================*/

class ClientCallbacks : public NimBLEClientCallbacks
{
    void onConnect(NimBLEClient* client) override
    {
        connected = true;
        keyState = KEY_CONNECTED;

        //Serial.println("[KEY BLE ] Connected");
    }

    void onDisconnect(NimBLEClient* client, int reason) override
    {
        if (connected)
        {
            Serial.println("[KEY BLE ] Disconnected");
        }

        connected = false;
        //memset(&keyState, 0, sizeof(keyState));
        keyState = KEY_SCANNING;
        doScan = true;
        pRemoteTx = nullptr;
        pRemoteRx = nullptr;
        pAdvertisedDevice = nullptr;
    }
};

static ClientCallbacks clientCallbacks;
/*=====================================================
                    SCAN CALLBACK
=====================================================*/

class AdvertisedCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice* device) override
    {
        if (!device->isAdvertisingService(
                NimBLEUUID(SERVICE_UUID)))
        {
            return;
        }

        int rssi = device->getRSSI();

        if (rssi < BLE_CONNECT_RSSI_THRESHOLD)
            return;

        Serial.printf("[KEY BLE ] Car found (RSSI=%d dBm)\n", rssi);

        NimBLEDevice::getScan()->stop();

        pAdvertisedDevice =
            const_cast<NimBLEAdvertisedDevice*>(device);
        doConnect = true;
    }
};

/*=====================================================
                    CONNECT
=====================================================*/

static bool ConnectToCar()
{
    if (pAdvertisedDevice == nullptr)
    {
        Serial.println("[KEY ERROR] No advertising device");
        return false;
    }
    
    if (pClient == nullptr)
    {
        pClient = NimBLEDevice::createClient();

        pClient->setClientCallbacks(
            &clientCallbacks,
            false);
    }

    if (!pClient->connect(pAdvertisedDevice))
    {
        Serial.println("[KEY ERROR] Connection failed");
        return false;
    }

    Serial.printf("[KEY BLE ] Connected (MTU=%d)\n",
              pClient->getMTU());

    NimBLERemoteService* service =
        pClient->getService(SERVICE_UUID);

    if (!service)
    {
        Serial.println("[KEY ERROR] Service not found");
        pClient->disconnect();
        return false;
    }

    Serial.println("[KEY BLE ] Service discovered");

    pRemoteTx = service->getCharacteristic(CHARACTERISTIC_UUID_TX);
    pRemoteRx = service->getCharacteristic(CHARACTERISTIC_UUID_RX);

    if (!pRemoteTx || !pRemoteRx)
    {
        Serial.println("[KEY BLE ] Characteristic not found");
        pClient->disconnect();
        return false;
    }
    Serial.println("[KEY BLE ] Characteristics discovered");
    if (!pRemoteTx->canNotify())
    {
        Serial.println("[KEY ERROR] TX characteristic does not support Notify");
        pClient->disconnect();
        return false;
    }

    bool ok = pRemoteTx->subscribe(true, NotifyCallback);

    if (!ok)
    {
        Serial.println("[KEY ERROR] Failed to enable notifications");
        pClient->disconnect();
        return false;
    }

    Serial.println("[KEY BLE ] Notifications enabled");

    Packet pkt = {};
    pkt.type = PKT_READY;
    pkt.length = 0;

    if (!BLE_Key_SendPacket(pkt))
    {
        Serial.println("[KEY ERROR] Failed to send READY");
        pClient->disconnect();
        return false;
    }

    keyState = KEY_READY;

    //Serial.println("[BLE KEY] READY sent");
    return true;
}

/*=====================================================
                    INIT
=====================================================*/

bool BLE_Key_Init()
{
    NimBLEDevice::init(DEVICE_NAME_KEYFOB);
    NimBLEDevice::setMTU(128);
    NimBLEScan* scan = NimBLEDevice::getScan();

    static AdvertisedCallbacks advertisedCallbacks;
    scan->setScanCallbacks(&advertisedCallbacks);
    scan->setActiveScan(true);
    scan->start(0, false);

    keyState = KEY_SCANNING;

    Serial.println("[KEY BLE ] Scanning...");
    return true;
}

/*=====================================================
                    TASK
=====================================================*/

void BLE_Key_Task()
{
    if (doConnect)
    {
        if (ConnectToCar())
        {
            Serial.println("[KEY AUTH] Waiting for challenge...");
        }
        else
        {
            Serial.println("[KEY ERROR] Connection failed");
            doScan = true;
        }

        doConnect = false;
    }

    if (doScan && !connected)
    {
        keyState = KEY_SCANNING;

        NimBLEDevice::getScan()->start(0, false);
        doScan = false;
    }
    static uint32_t lastRssiLog = 0;
    
    // if (connected && pClient != nullptr)
    // {
    //     if (millis() - lastRssiLog > 5000) 
    //     {
    //         lastRssiLog = millis();
    //         int currentRssi = pClient->getRssi();
            
    //         Serial.printf("[BLE] Cường độ sóng (RSSI) = %d dBm\n", currentRssi);

    //         // Bạn có thể thêm logic mô phỏng ngắt kết nối nếu đi quá xa
    //         // if (currentRssi < -85) {
    //         //     Serial.println("[WARNING] Bạn đang ra khỏi vùng phủ sóng!");
    //         // }
    //     }
    // }
}

/*=====================================================
                    STATUS
=====================================================*/

bool BLE_Key_IsConnected()
{
    return connected;
}

/*=====================================================
                    SEND PACKET
=====================================================*/

bool BLE_Key_SendPacket(const Packet& pkt)
{
    if (!connected)
        return false;

    if (pkt.type == PKT_RESPONSE)
    {
        if (keyState != KEY_CHALLENGE_RECEIVED)
        {
            Serial.println("[KEY ERROR] Challenge not received");
            return false;
        }
    }

    if (!pRemoteRx)
        return false;

    if (pkt.length > MAX_PACKET_SIZE)
        return false;

    if (pkt.type == 0)
        return false;

    uint8_t buffer[2 + MAX_PACKET_SIZE];

    buffer[0] = (uint8_t)pkt.type;
    buffer[1] = pkt.length;

    if (pkt.length > 0)
    {
        memcpy(buffer + 2,
            pkt.data,
            pkt.length);
    }

    bool ok = pRemoteRx->writeValue(
        buffer,
        pkt.length + 2,
        false);
    
    if (ok && pkt.type == PKT_RESPONSE)
    {
        //keyState = KEY_READY;
    }

    Serial.printf("[KEY TX  ] %s %s\n",
              PacketTypeToString(pkt.type),
              ok ? "OK" : "FAILED");

    return ok;
}