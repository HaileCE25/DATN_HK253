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

constexpr size_t KEY_REQUEST_PAYLOAD_SIZE = 16;
constexpr size_t KEY_RESPONSE_PAYLOAD_SIZE = 33;

struct KeyRequestPayload
{
    char car_id[16];
};

struct KeyResponsePayload
{
    uint8_t key_root[32];
    uint8_t key_len;
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
