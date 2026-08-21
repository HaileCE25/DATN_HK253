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

// Cooldown: sau mỗi lần disconnect (bất kể lý do), car ngừng advertising
// trong BLE_COOLDOWN_MS trước khi advertise lại. onDisconnect() không
// phân biệt được disconnect do fail hay bình thường, nên áp dụng đồng
// nhất cho mọi lần disconnect - đơn giản và vẫn đạt mục tiêu chống
// vòng lặp found->connect->disconnect liên tục (xem log Test 1).
static bool advertising = false;
static uint32_t cooldownUntil = 0;

// Chỉ cho phép ĐÚNG 1 kết nối tại 1 thời điểm - nếu không giới hạn,
// bất kỳ thiết bị BLE nào (kể cả không đúng Keyfob) cũng kết nối được
// song song, gây: (1) carState/savedNonce (biến toàn cục dùng chung)
// bị nhiều kết nối ghi đè lẫn nhau, (2) notify() mặc định gửi tới MỌI
// thiết bị đang subscribe - kẻ lạ có thể "nghe lén" CHALLENGE dù không
// giải mã được (không có key_root, nhưng vẫn thấy được luồng trao
// đổi). Đã xác nhận qua thực nghiệm bằng nRF Connect.
static bool s_hasActiveConnection = false;

/*==================== CALLBACK ====================*/

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override
    {
        if (s_hasActiveConnection)
        {
            // Đã có 1 kết nối khác đang hoạt động - từ chối ngay,
            // KHÔNG đụng tới bất kỳ biến trạng thái nào của phiên
            // đang chạy (deviceConnected, carState... giữ nguyên).
            LOG_PRINTLN("[CAR BLE  ] Da co ket noi khac dang hoat dong - tu choi ket noi moi");
            pSrv->disconnect(connInfo.getConnHandle());
            return;
        }

        s_hasActiveConnection = true;

        deviceConnected = true;
        advertising = false; // NimBLE tự dừng advertising khi có connection
        carState = CAR_CONNECTED;
        NimBLEDevice::getServer()->updateConnParams(connInfo.getConnHandle(), 16, 32, 0, 400);
        LOG_PRINTLN("[CAR BLE  ] Connected");
    }

    // Nơi DUY NHẤT đưa carState về CAR_IDLE và dọn toàn bộ session state.
    // KHÔNG advertise lại ngay ở đây - chỉ đặt mốc thời gian cooldown.
    // Việc advertise lại được BLE_Car_Task() polling và thực hiện sau
    // khi cooldown hết hạn.
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override
    {
        s_hasActiveConnection = false;

        deviceConnected = false;
        carState = CAR_IDLE;

        cooldownUntil = millis() + BLE_COOLDOWN_MS;

        LOG_PRINTF("[CAR BLE  ] Disconnected - cooldown %lu ms\n",
                      (unsigned long)BLE_COOLDOWN_MS);
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
            LOG_PRINTLN("[CAR ERROR] RX queue full");
        }

        LOG_PRINTF("[CAR RX   ] %s\n",
              PacketTypeToString(pkt.type));

        /*================ TRANSPORT-LEVEL STATE =================
         * Callback chỉ set state phản ánh SỰ KIỆN TRANSPORT đã xảy ra,
         * KHÔNG set state phản ánh KẾT QUẢ NGHIỆP VỤ. Việc set
         * CAR_CHALLENGE_SENT / CAR_AUTHENTICATED do TaskLogic gọi qua
         * BLE_Car_SetState() sau khi thực sự xử lý xong.
         */
        switch (pkt.type)
        {
        case PKT_READY:
            carState = CAR_KEY_RECEIVED;
            break;

        default:
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
        LOG_PRINTLN("[CAR ERROR] Create characteristic failed");
        return false;
    }

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->start();
    advertising = true;

    LOG_PRINTLN("[CAR BLE  ] Advertising");
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
        LOG_PRINTLN("[CAR ERROR] Notify failed");
        return false;
    }
    LOG_PRINTF("[CAR TX   ] %s\n",
              PacketTypeToString(pkt.type));
    return true;
}

/*==================== DISCONNECT ====================*/

bool BLE_Car_Disconnect()
{
    if (!deviceConnected || pServer == nullptr)
        return false;

    if (pServer->getConnectedCount() == 0)
        return false;

    std::vector<uint16_t> peerIds = pServer->getPeerDevices();
    if (peerIds.empty())
        return false;

    pServer->disconnect(peerIds[0]);
    // Không set carState ở đây - onDisconnect() sẽ lo việc đó
    // khi NimBLE stack xác nhận ngắt xong.
    return true;
}

/*==================== STATE ACCESSORS ====================*/

void BLE_Car_SetState(CarState newState)
{
    carState = newState;
}

CarState BLE_Car_GetState()
{
    return carState;
}

/*==================== TASK ====================*/

void BLE_Car_Task()
{
    // Hết cooldown, chưa connect, chưa advertise -> advertise lại.
    if (!deviceConnected && !advertising &&
        millis() >= cooldownUntil)
    {
        NimBLEDevice::startAdvertising();
        advertising = true;
        LOG_PRINTLN("[CAR BLE  ] Advertising");
    }
}

bool BLE_Car_IsConnected()
{
    return deviceConnected;
}