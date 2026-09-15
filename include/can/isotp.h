#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <driver/twai.h>

// -----------------------------------------------------------------------------
// ISO-TP (ISO 15765-2) - đơn giản hóa cho hệ thống 2 node (Car <-> Gateway)
// -----------------------------------------------------------------------------
// Đơn giản hóa so với chuẩn đầy đủ: dùng CHUNG 1 CAN ID cho cả chiều dữ
// liệu (FF/CF từ bên gửi) VÀ Flow Control (FC từ bên nhận) của cùng 1
// luồng - hợp lý vì bus chỉ có 2 node, phân biệt qua PCI nibble (không
// cần ID riêng cho FC như mạng nhiều ECU thật).
// Flow Control đơn giản: luôn "Clear To Send" toàn bộ 1 lần (block
// size=0, STmin=0) - đủ cho payload nhỏ (vài chục byte) của hệ thống này.

// Gửi 1 payload (có thể > 8 byte) qua CAN, tự động chia SF/FF/CF theo
// ISO-TP, chờ Flow Control nếu cần nhiều frame. Blocking tối đa timeoutMs.
bool ISOTP_Send(
    uint32_t can_id,
    const uint8_t* data,
    size_t length,
    uint32_t timeoutMs);

// Nhận 1 payload qua CAN, tự ráp lại SF/FF/CF, tự gửi Flow Control khi
// cần. Blocking tối đa timeoutMs cho toàn bộ quá trình.
bool ISOTP_Receive(
    uint32_t can_id,
    uint8_t* outBuffer,
    size_t bufferCapacity,
    size_t& outLength,
    uint32_t timeoutMs);

// Giống ISOTP_Receive nhưng frame ĐẦU TIÊN (SF hoặc FF) đã được nơi gọi đọc
// ra khỏi bus. Dùng cho node nhận nhiều CAN ID: 1 vòng dispatch đọc frame
// rồi route theo identifier, thay vì mỗi handler tự đọc bus và vô tình vứt
// bỏ frame của ID khác. CAN ID của luồng lấy từ firstFrame.identifier.
// SF trả về ngay; FF thì gửi Flow Control và chờ CF tối đa timeoutMs.
bool ISOTP_ReceiveFromFirstFrame(
    const twai_message_t& firstFrame,
    uint8_t* outBuffer,
    size_t bufferCapacity,
    size_t& outLength,
    uint32_t timeoutMs);