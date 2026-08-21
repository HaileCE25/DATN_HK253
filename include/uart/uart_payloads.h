#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// UART Payloads (application data, độc lập với UartFrame transport)
// -----------------------------------------------------------------------------
// Cùng nguyên tắc đã áp dụng cho CAN's KeyProvisionPayload: struct chỉ mô
// tả dữ liệu trong RAM, KHÔNG dùng sizeof()/memcpy trực tiếp để đưa lên
// wire - Serialize/Deserialize tự định nghĩa wire format và tự validate
// buffer trước khi đọc/ghi.

constexpr size_t KEY_REQUEST_PAYLOAD_SIZE = 16;   // car_id, null-padded
constexpr size_t KEY_RESPONSE_PAYLOAD_SIZE = 33;  // 32 byte key + 1 byte key_len

struct KeyRequestPayload
{
    char car_id[16]; // null-terminated, tối đa 15 ký tự + '\0'
};

struct KeyResponsePayload
{
    uint8_t key_root[32];
    uint8_t key_len; // độ dài thật của key (<=32) - Firebase hiện trả 16 byte
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