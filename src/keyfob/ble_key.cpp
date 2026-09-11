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

// car_id của chính Keyfob này (gán qua BLE_Key_SetOwnCarId lúc boot) -
// gửi kèm trong gói READY để Car so khớp trước khi tiếp tục.
static char s_ownCarId[16] = {0};

// Khi true, onDisconnect() KHÔNG tự kích hoạt scan lại - dùng cho lỗi
// vĩnh viễn (car_id sai), xem BLE_Key_HaltPermanently().
static bool s_permanentlyHalted = false;

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

    LOG_PRINTF("[KEY RX  ] %s\n",
              PacketTypeToString(pkt.type));

    if (xQueueSend(bleRxQueue, &pkt, 0) != pdPASS)
    {
        LOG_PRINTLN("[KEY ERROR] RX queue full");
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
    }

    void onDisconnect(NimBLEClient* client, int reason) override
    {
        if (connected)
        {
            LOG_PRINTLN("[KEY BLE ] Disconnected");
        }

        connected = false;
        keyState = KEY_SCANNING;

        if (!s_permanentlyHalted)
        {
            doScan = true;
        }
        else
        {
            LOG_PRINTLN("[KEY BLE ] Đã dừng hoạt động do car_id sai. Không tự scan lại");
        }

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

        LOG_PRINTF("[KEY BLE ] Car found (RSSI=%d dBm)\n", rssi);

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
        LOG_PRINTLN("[KEY ERROR] No advertising device");
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
        LOG_PRINTLN("[KEY ERROR] Connection failed");
        return false;
    }

    LOG_PRINTF("[KEY BLE ] Connected (MTU=%d)\n",
              pClient->getMTU());

    NimBLERemoteService* service =
        pClient->getService(SERVICE_UUID);

    if (!service)
    {
        LOG_PRINTLN("[KEY ERROR] Service not found");
        pClient->disconnect();
        return false;
    }

    LOG_PRINTLN("[KEY BLE ] Service discovered");

    pRemoteTx = service->getCharacteristic(CHARACTERISTIC_UUID_TX);
    pRemoteRx = service->getCharacteristic(CHARACTERISTIC_UUID_RX);

    if (!pRemoteTx || !pRemoteRx)
    {
        LOG_PRINTLN("[KEY BLE ] Characteristic not found");
        pClient->disconnect();
        return false;
    }
    LOG_PRINTLN("[KEY BLE ] Characteristics discovered");

    if (!pRemoteTx->canNotify())
    {
        LOG_PRINTLN("[KEY ERROR] TX characteristic does not support Notify");
        pClient->disconnect();
        return false;
    }

    bool ok = pRemoteTx->subscribe(true, NotifyCallback);

    if (!ok)
    {
        LOG_PRINTLN("[KEY ERROR] Failed to enable notifications");
        pClient->disconnect();
        return false;
    }

    LOG_PRINTLN("[KEY BLE ] Notifications enabled");

    // Gửi READY kèm car_id của chính Keyfob này - Car sẽ so khớp
    // trước khi tiếp tục (xem car/main.cpp case PKT_READY).
    Packet pkt = {};
    pkt.type = PKT_READY;
    pkt.length = (uint8_t)strlen(s_ownCarId);
    memcpy(pkt.data, s_ownCarId, pkt.length);

    if (!BLE_Key_SendPacket(pkt))
    {
        LOG_PRINTLN("[KEY ERROR] Failed to send READY");
        pClient->disconnect();
        return false;
    }

    keyState = KEY_READY;

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

    LOG_PRINTLN("[KEY BLE ] Scanning...");
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
            LOG_PRINTLN("[KEY AUTH] Waiting for challenge...");
        }
        else
        {
            LOG_PRINTLN("[KEY ERROR] Connection failed");
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
}

/*=====================================================
                    STATUS
=====================================================*/

bool BLE_Key_IsConnected()
{
    return connected;
}

/*=====================================================
                    DISCONNECT
=====================================================*/

bool BLE_Key_Disconnect()
{
    if (!connected || pClient == nullptr)
        return false;

    pClient->disconnect();
    return true;
}

/*=====================================================
                    STATE ACCESSORS
=====================================================*/

void BLE_Key_SetState(KeyState newState)
{
    keyState = newState;
}

KeyState BLE_Key_GetState()
{
    return keyState;
}

void BLE_Key_SetOwnCarId(const char* carId)
{
    strncpy(s_ownCarId, carId, sizeof(s_ownCarId) - 1);
}

void BLE_Key_HaltPermanently()
{
    s_permanentlyHalted = true;

    if (connected && pClient != nullptr)
    {
        pClient->disconnect(); // onDisconnect() sẽ tự thấy cờ này, không scan lại
    }
    else
    {
        NimBLEDevice::getScan()->stop();
        doScan = false;
    }

    //LOG_PRINTLN("[KEY BLE ] HALT vĩnh viễn - car_id không khớp, kiểm tra lại provisioning");
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
            LOG_PRINTLN("[KEY ERROR] Challenge not received");
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

    LOG_PRINTF("[KEY TX  ] %s %s\n",
              PacketTypeToString(pkt.type),
              ok ? "OK" : "FAILED");

    return ok;
}