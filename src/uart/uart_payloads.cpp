#include "uart/uart_payloads.h"
#include <string.h>

bool SerializeKeyRequest(
    const KeyRequestPayload& payload,
    uint8_t* buffer,
    size_t buffer_capacity)
{
    if (buffer == nullptr)
        return false;

    if (buffer_capacity < KEY_REQUEST_PAYLOAD_SIZE)
        return false;

    memset(buffer, 0, KEY_REQUEST_PAYLOAD_SIZE);
    strncpy((char*)buffer, payload.car_id, KEY_REQUEST_PAYLOAD_SIZE - 1);

    return true;
}

bool DeserializeKeyRequest(
    const uint8_t* buffer,
    size_t buffer_length,
    KeyRequestPayload& payload)
{
    if (buffer == nullptr)
        return false;

    if (buffer_length != KEY_REQUEST_PAYLOAD_SIZE)
        return false;

    memcpy(payload.car_id, buffer, KEY_REQUEST_PAYLOAD_SIZE);
    payload.car_id[KEY_REQUEST_PAYLOAD_SIZE - 1] = '\0'; // đảm bảo null-terminated

    return true;
}

bool SerializeKeyResponse(
    const KeyResponsePayload& payload,
    uint8_t* buffer,
    size_t buffer_capacity)
{
    if (buffer == nullptr)
        return false;

    if (buffer_capacity < KEY_RESPONSE_PAYLOAD_SIZE)
        return false;

    if (payload.key_len > 32)
        return false;

    memcpy(buffer, payload.key_root, 32);
    buffer[32] = payload.key_len;

    return true;
}

bool DeserializeKeyResponse(
    const uint8_t* buffer,
    size_t buffer_length,
    KeyResponsePayload& payload)
{
    if (buffer == nullptr)
        return false;

    if (buffer_length != KEY_RESPONSE_PAYLOAD_SIZE)
        return false;

    uint8_t keyLen = buffer[32];
    if (keyLen > 32)
        return false;

    memcpy(payload.key_root, buffer, 32);
    payload.key_len = keyLen;

    return true;
}