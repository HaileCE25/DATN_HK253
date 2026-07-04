#pragma once
#include <Arduino.h>
#include "shared/config.h"

struct Packet; 

bool BLE_Car_Init();
void BLE_Car_Task();
bool BLE_Car_IsConnected();

bool BLE_Car_SendPacket(const Packet& pkt);

enum CarState
{
    CAR_IDLE = 0,
    CAR_CONNECTED,
    CAR_KEY_RECEIVED,
    CAR_CHALLENGE_SENT,
    CAR_AUTHENTICATED
};