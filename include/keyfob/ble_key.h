#ifndef BLE_KEY_H
#define BLE_KEY_H

#include <Arduino.h>
#include "shared/config.h"
#include "protocol/packet.h"

/*==================== API ====================*/

bool BLE_Key_Init();
void BLE_Key_Task();
bool BLE_Key_IsConnected();
bool BLE_Key_SendPacket(const Packet& pkt);

/*==================== STATE ====================*/

enum KeyState
{
    KEY_IDLE = 0,
    KEY_SCANNING,
    KEY_CONNECTED,
    KEY_READY,              
    KEY_CHALLENGE_RECEIVED,
    KEY_AUTHENTICATED
};

#endif