#pragma once

#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// TWAI Driver Wrapper
// -----------------------------------------------------------------------------
// Tầng transport thô nhất: chỉ gửi/nhận 1 CAN frame (tối đa 8 byte).
// KHÔNG biết gì về ISO-TP, payload nghiệp vụ, hay KEY_ROOT.
// Dùng chung cho cả car (Access ECU) và gateway - hành vi khác nhau
// (GPIO, vai trò REQ/RESP) do lớp phía trên (isotp.h, main.cpp) quyết định.

// Khởi tạo và start TWAI driver. Idempotent - gọi nhiều lần vẫn an toàn.
bool TWAI_Init();

// Gửi 1 frame thô. Tự validate dlc <= 8 và data != nullptr (khi dlc > 0).
bool TWAI_Send(
    uint32_t id,
    const uint8_t* data,
    uint8_t dlc);

// Nhận 1 frame thô. timeout = 0 -> non-blocking (poll).
// timeout > 0 -> blocking tối đa timeout ticks.
// Chính sách blocking/non-blocking do nơi gọi quyết định, driver không
// áp đặt.
bool TWAI_Receive(
    twai_message_t& message,
    TickType_t timeout = 0);