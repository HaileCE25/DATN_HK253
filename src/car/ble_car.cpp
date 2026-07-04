#include "car/ble_car.h"
#include <NimBLEDevice.h>
#include "shared/config.h"
#include "protocol/packet.h"
#include "os/queue.h"

static NimBLEServer* pServer = nullptr;
static NimBLECharacteristic* pTxCharacteristic = nullptr;
static NimBLECharacteristic* pRxCharacteristic = nullptr;

static bool deviceConnected = false;
static CarState carState = CAR_IDLE;
static uint8_t currentNonce[16];
static uint32_t challengeSendTime = 0;
static void generate_challenge(uint8_t* out);

static void generate_challenge(uint8_t* out)
{
    esp_fill_random(out, 16);
}

/*==================== CALLBACK ====================*/

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override
    {
        deviceConnected = true;
        carState = CAR_CONNECTED;
        NimBLEDevice::getServer()->updateConnParams(connInfo.getConnHandle(), 16, 32, 0, 400);
        Serial.println("[CAR BLE  ] Connected");
    }

    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override
    {
        deviceConnected = false;
        carState = CAR_IDLE;
        memset(currentNonce, 0, sizeof(currentNonce));
        Serial.println("[CAR BLE  ] Disconnected");
        NimBLEDevice::startAdvertising();
    }
};

/*==================== RX ====================*/

class RXCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) override
    {
        std::string v = pRxCharacteristic->getValue();

        if (v.size() < 2) return;

        uint8_t type = v[0];
        uint8_t len  = v[1];

        if (len > MAX_PACKET_SIZE) return;
        if (v.size() < 2 + len) return;

        Packet pkt = {};
        pkt.type = (PacketType)type;
        pkt.length = len;
        memcpy(pkt.data, v.data() + 2, len);

        if (xQueueSend(bleRxQueue, &pkt, 0) != pdPASS)
        {
            Serial.println("[CAR ERROR] RX queue full");
        }

        Serial.printf("[CAR RX   ] %s\n",
              PacketTypeToString(pkt.type));

        /*================ STATE MACHINE =================*/

        switch(pkt.type)
        {
        case PKT_READY:

            carState = CAR_KEY_RECEIVED;

            //Serial.println("[CAR RX   ] READY");

            break;

        case PKT_RESPONSE:
            carState = CAR_AUTHENTICATED;
            //Serial.println("[CAR RX   ] Response");

            break;
        }
    }
};

/*==================== INIT ====================*/

bool BLE_Car_Init()
{
    NimBLEDevice::init(DEVICE_NAME_CAR);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* service = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        NIMBLE_PROPERTY::NOTIFY);

    pRxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        NIMBLE_PROPERTY::WRITE);

    pRxCharacteristic->setCallbacks(new RXCallbacks());
    if (!pTxCharacteristic || !pRxCharacteristic)
    {
        Serial.println("[CAR ERROR] Create characteristic failed");
        return false;
    }
    //service->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();

    Serial.println("[CAR BLE  ] Advertising");
    return true;
}

/*==================== SEND ====================*/

bool BLE_Car_SendPacket(const Packet& pkt)
{
    if (!deviceConnected) return false;
    if (carState == CAR_IDLE) return false;
    if (pkt.length > MAX_PACKET_SIZE) return false;
    if (pkt.type == 0) return false;

    uint8_t buffer[2 + MAX_PACKET_SIZE];

    buffer[0] = (uint8_t)pkt.type;
    buffer[1] = pkt.length;
    if (pkt.length > 0)
    {
        memcpy(buffer + 2,
            pkt.data,
            pkt.length);
    }

    pTxCharacteristic->setValue(buffer, pkt.length + 2);
    bool ok = pTxCharacteristic->notify();
    if (!ok)
    {
        Serial.println("[CAR ERROR] Notify failed");
        return false;
    }
    Serial.printf("[CAR TX   ] %s\n",
              PacketTypeToString(pkt.type));
    return true;
}

/*==================== TASK ====================*/

void BLE_Car_Task()
{
    //Serial.printf("[CAR] state=%d\n", carState);
    // if (carState != CAR_KEY_RECEIVED)
    //     return;

    // Packet pkt = {};

    // pkt.type = PKT_CHALLENGE;
    // pkt.length = 16;

    // generate_challenge(currentNonce);
    // memcpy(pkt.data, currentNonce, 16);

    // if (BLE_Car_SendPacket(pkt))
    // {
    //     carState = CAR_CHALLENGE_SENT;
    //     challengeSendTime = millis();
    //     Serial.println("[CAR] Challenge sent");
    // }

    if (carState == CAR_CHALLENGE_SENT)
    {
        if (millis() - challengeSendTime > 3000)
        {
            Serial.println("[CAR AUTH ] Response timeout");
            if (pServer->getConnectedCount() > 0) {
                std::vector<uint16_t> peerIds = pServer->getPeerDevices();
                if (!peerIds.empty()) {
                    pServer->disconnect(peerIds[0]); 
                }
            }
            carState = CAR_IDLE;
        }
    }
}

bool BLE_Car_IsConnected()
{
    return deviceConnected;
}
