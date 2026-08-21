#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// CAN Arbitration IDs
// -----------------------------------------------------------------------------
// Tên hằng số mô tả LOẠI message trong protocol, không mô tả ai gửi.
// Hướng truyền được quy định riêng, không nằm trong tên biến:
//   REQ  : Car (Access ECU) -> Gateway ECU
//   RESP : Gateway ECU -> Car (Access ECU)

// Car yêu cầu Gateway cấp KEY_ROOT
constexpr uint32_t CAN_ID_KEY_PROVISION_REQ = 0x100;

// Gateway trả về payload chứa KEY_ROOT
constexpr uint32_t CAN_ID_KEY_PROVISION_RESP = 0x101;