#pragma once
#include <Arduino.h>
#include "shared/config.h"

struct Packet;

enum CarState
{
    CAR_IDLE = 0,
    CAR_CONNECTED,
    CAR_KEY_RECEIVED,
    CAR_CHALLENGE_SENT,
    CAR_AUTHENTICATED
};

bool BLE_Car_Init();
void BLE_Car_Task();
bool BLE_Car_IsConnected();
bool BLE_Car_SendPacket(const Packet& pkt);

// Yêu cầu ngắt kết nối phiên BLE hiện tại (nếu có).
// Việc dọn state (carState = CAR_IDLE, xoá nonce, advertise lại)
// KHÔNG xảy ra ngay ở đây — nó xảy ra trong ServerCallbacks::onDisconnect
// khi NimBLE stack xác nhận đã ngắt xong. Đây là nơi DUY NHẤT đưa
// carState về CAR_IDLE, để tránh state bị set rải rác nhiều chỗ.
bool BLE_Car_Disconnect();

// Transition trạng thái ở tầng PROTOCOL/BUSINESS LOGIC (do TaskLogic gọi),
// khác với transition ở tầng TRANSPORT (onConnect/onDisconnect/onWrite)
// vốn được ble_car.cpp tự quản lý nội bộ.
// Ví dụ: sau khi gửi CHALLENGE thành công -> CAR_CHALLENGE_SENT.
//        sau khi verify HMAC đúng -> CAR_AUTHENTICATED.
void BLE_Car_SetState(CarState newState);
CarState BLE_Car_GetState();