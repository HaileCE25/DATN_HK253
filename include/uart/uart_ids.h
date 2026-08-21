#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// UART Message Types (Car <-> Gateway)
// -----------------------------------------------------------------------------
// Tách riêng khỏi protocol/packet.h (PacketType) - đó là application-layer
// message riêng của BLE. UART giữa Car và Gateway là protocol khác hoàn
// toàn, không liên quan tới BLE auth flow.

enum UartMsgType : uint8_t
{
    UART_MSG_KEY_REQUEST  = 0x01,   // Car -> Gateway: xin key_root theo car_id
    UART_MSG_KEY_RESPONSE = 0x02,   // Gateway -> Car: trả key_root
    UART_MSG_ERROR        = 0x03,   // Gateway -> Car: lookup Firebase thất bại
};