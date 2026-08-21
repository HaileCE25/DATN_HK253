#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include "shared/config.h"

/*=====================================================
                    Packet Type
=====================================================*/
enum AuthFailReason : uint8_t
{
    AUTH_FAIL_REASON_UNKNOWN = 0,
    AUTH_FAIL_REASON_CAR_ID_MISMATCH = 1,  // VĨNH VIỄN - dừng hẳn
    AUTH_FAIL_REASON_HMAC_INVALID = 2,     // Có thể tự hết - vẫn thử lại
    AUTH_FAIL_REASON_INVALID_LENGTH = 3,   // Có thể tự hết - vẫn thử lại
};

enum PacketType : uint8_t
{
    PKT_READY      = 0x01,   // Key -> Car

    PKT_CHALLENGE  = 0x02,   // Car -> Key

    PKT_RESPONSE   = 0x03,   // Key -> Car (HMAC)

    PKT_AUTH_OK    = 0x04,   // Car -> Key

    PKT_AUTH_FAIL  = 0x05,   // Car -> Key

    PKT_UNLOCK     = 0x06,   // Sau UWB thành công

    PKT_LOCK       = 0x07,   // Khóa xe

    PKT_ERROR      = 0x08    // Báo lỗi
};

/*=====================================================
                INTERNAL PACKET (SAFE)
=====================================================*/

struct Packet
{
    PacketType type;
    uint8_t length;
    uint8_t data[MAX_PACKET_SIZE];
};

const char* PacketTypeToString(PacketType type);
#endif