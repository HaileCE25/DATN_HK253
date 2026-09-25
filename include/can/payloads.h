#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// CAN Payloads (application data cho ISO-TP)
// -----------------------------------------------------------------------------
// Cùng nội dung với uart/uart_payloads.h nhưng ĐỘC LẬP - can/ không phụ
// thuộc uart/ (2 subsystem tách biệt, dù wire format giống nhau vì cùng
// mô tả 1 nghiệp vụ KEY_ROOT provisioning).

constexpr size_t KEY_REQUEST_PAYLOAD_SIZE = 17;
constexpr size_t KEY_RESPONSE_PAYLOAD_SIZE = 38;

// expire_unix = 0: Gateway không có thông tin hạn (Firebase chưa có expire_time
// hoặc không đọc được). Car coi là "không rõ", không phải "không bao giờ hết hạn".
constexpr uint32_t KEY_EXPIRE_UNKNOWN = 0;

// Trạng thái quyền dùng key của xe, Gateway xác định từ /Bookings.
// KEY_STATUS_REVOKED: tra cứu THÀNH CÔNG nhưng xe không còn booking ACTIVE
// (admin thu hồi -> REVOKED, hoặc chưa/không còn booking). Khi đó key_root
// trong payload để toàn 0, Car phải xoá key đang giữ và huỷ phiên.
// Tra cứu lỗi (mất mạng, Firebase lỗi) thì Gateway KHÔNG trả lời - Car coi
// là "không rõ" và giữ nguyên key cũ, không coi là bị thu hồi.
constexpr uint8_t KEY_STATUS_OK      = 0x00;
constexpr uint8_t KEY_STATUS_REVOKED = 0x01;

// Wire format:  [0..15]  car_id (kết thúc '\0')
//               [16]     revalidate: 1 = lượt Car hỏi lại định kỳ khi đã có key
//                        (Gateway xử lý y hệt, chỉ không in log/debug)
struct KeyRequestPayload
{
    char car_id[16];
    bool revalidate;
};

// Wire format:  [0..31]  key_root
//               [32]     key_len
//               [33..36] expire_unix (UTC), little-endian
//               [37]     status: KEY_STATUS_*
struct KeyResponsePayload
{
    uint8_t  key_root[32];
    uint8_t  key_len;
    uint32_t expire_unix;
    uint8_t  status;
};

bool SerializeKeyRequest(
    const KeyRequestPayload& payload,
    uint8_t* buffer,
    size_t buffer_capacity);

bool DeserializeKeyRequest(
    const uint8_t* buffer,
    size_t buffer_length,
    KeyRequestPayload& payload);

bool SerializeKeyResponse(
    const KeyResponsePayload& payload,
    uint8_t* buffer,
    size_t buffer_capacity);

bool DeserializeKeyResponse(
    const uint8_t* buffer,
    size_t buffer_length,
    KeyResponsePayload& payload);

// -----------------------------------------------------------------------------
// Actuator command payload (CAN_ID_ACTUATOR_CMD, 1 byte)
// -----------------------------------------------------------------------------
constexpr uint8_t ACTUATOR_CMD_UNLOCK = 0x01; // Mở khoá
constexpr uint8_t ACTUATOR_CMD_LOCK   = 0x02; // Khoá lại

// -----------------------------------------------------------------------------
// Car status payload (CAN_ID_CAR_STATUS, 4 byte - Single Frame)
// -----------------------------------------------------------------------------
// Wire format:  [0] ble   : CAR_STATUS_BLE_*
//               [1] fsm   : CAR_STATUS_FSM_*
//               [2..3]    : distance_cm, little-endian (CAR_STATUS_DISTANCE_INVALID = chưa có mẫu)
// Giá trị enum là HẰNG SỐ WIRE riêng, không phải CarState/CarFsmState_t nội bộ
// của car - car tự map sang, để đổi thứ tự enum nội bộ không làm hỏng gateway.
constexpr size_t   CAR_STATUS_PAYLOAD_SIZE     = 4;
constexpr uint16_t CAR_STATUS_DISTANCE_INVALID = 0xFFFF;

constexpr uint8_t CAR_STATUS_BLE_ADVERTISING    = 0x00; // chưa có keyfob kết nối
constexpr uint8_t CAR_STATUS_BLE_CONNECTED      = 0x01; // đã kết nối, chưa gửi READY
constexpr uint8_t CAR_STATUS_BLE_AUTHENTICATING = 0x02; // READY/CHALLENGE đang xử lý
constexpr uint8_t CAR_STATUS_BLE_AUTHENTICATED  = 0x03; // HMAC hợp lệ
constexpr uint8_t CAR_STATUS_BLE_AUTH_FAIL      = 0x04; // vừa bị từ chối (giữ vài giây để kịp hiển thị)
constexpr uint8_t CAR_STATUS_BLE_MAX            = CAR_STATUS_BLE_AUTH_FAIL;

constexpr uint8_t CAR_STATUS_FSM_IDLE          = 0x00;
constexpr uint8_t CAR_STATUS_FSM_AUTH          = 0x01;
constexpr uint8_t CAR_STATUS_FSM_TRACKING      = 0x02;
constexpr uint8_t CAR_STATUS_FSM_UNLOCK_WINDOW = 0x03;
constexpr uint8_t CAR_STATUS_FSM_UNLOCKED      = 0x04;
constexpr uint8_t CAR_STATUS_FSM_COOLDOWN      = 0x05;
constexpr uint8_t CAR_STATUS_FSM_MAX           = CAR_STATUS_FSM_COOLDOWN;

struct CarStatusPayload
{
    uint8_t  ble;
    uint8_t  fsm;
    uint16_t distance_cm;
};

bool SerializeCarStatus(
    const CarStatusPayload& payload,
    uint8_t* buffer,
    size_t buffer_capacity);

bool DeserializeCarStatus(
    const uint8_t* buffer,
    size_t buffer_length,
    CarStatusPayload& payload);
