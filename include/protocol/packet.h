#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include "shared/config.h"

/*=====================================================
                    Packet Type
=====================================================*/

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