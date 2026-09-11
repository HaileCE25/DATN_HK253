#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// UART Transport (Car <-> Gateway) - TẠM THAY CAN để test, sẽ quay lại CAN
// sau khi debug xong phần cứng (xem ghi chú trong can/).
// -----------------------------------------------------------------------------
// Framing đơn giản: [type(1)][length(1)][data(length)] - giống cấu trúc
// Packet của BLE, nhưng dùng UartMsgType riêng (uart_ids.h), không dùng
// chung PacketType.

constexpr uint8_t UART_MAX_PAYLOAD = 64; // đủ cho KeyResponsePayload (33 byte)

struct UartFrame
{
    uint8_t type;
    uint8_t length;
    uint8_t data[UART_MAX_PAYLOAD];
};

// Khởi tạo UART1 theo cấu hình trong uart_config.h. Idempotent.
bool UART_Init();

// Gửi 1 frame. Tự validate frame.length <= UART_MAX_PAYLOAD.
bool UART_SendFrame(const UartFrame& frame);

// Nhận 1 frame, blocking tối đa timeoutMs. Trả false nếu timeout hoặc
// frame nhận được không hợp lệ (length vượt UART_MAX_PAYLOAD).
bool UART_ReceiveFrame(UartFrame& outFrame, uint32_t timeoutMs);