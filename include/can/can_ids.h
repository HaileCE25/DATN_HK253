#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// CAN Arbitration IDs
// -----------------------------------------------------------------------------
// Tên hằng số mô tả LOẠI message trong protocol, không mô tả ai gửi.
// Hướng truyền được quy định riêng, không nằm trong tên biến:
//   REQ  : Car (Access ECU) -> Gateway ECU
//   RESP : Gateway ECU -> Car (Access ECU)
//   CMD  : Car (Access ECU) -> Gateway ECU (forward tới actuator)

// Car yêu cầu Gateway cấp KEY_ROOT
constexpr uint32_t CAN_ID_KEY_PROVISION_REQ = 0x100;

// Gateway trả về payload chứa KEY_ROOT
constexpr uint32_t CAN_ID_KEY_PROVISION_RESP = 0x101;

// Car ra lệnh mở/khoá actuator (1-byte payload: ACTUATOR_CMD_*)
// Gateway nhận và forward tín hiệu tới relay/actuator vật lý.
constexpr uint32_t CAN_ID_ACTUATOR_CMD = 0x200;
