#ifndef BLE_KEY_H
#define BLE_KEY_H

#include <Arduino.h>
#include "shared/config.h"
#include "protocol/packet.h"

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

/*==================== API ====================*/

bool BLE_Key_Init();
void BLE_Key_Task();
bool BLE_Key_IsConnected();
bool BLE_Key_SendPacket(const Packet& pkt);
bool BLE_Key_Disconnect();
void BLE_Key_SetState(KeyState newState);
KeyState BLE_Key_GetState();

// Gán car_id CỦA CHÍNH XE mà Keyfob này được cấp phát (đọc từ NVS lúc
// boot, gọi 1 lần trong setup()) - dùng để gửi kèm trong gói READY,
// giúp Car sớm phát hiện Keyfob sai xe trước khi tốn công sinh
// CHALLENGE/xin key_root. Đây CHỈ là bộ lọc nhanh (tối ưu), KHÔNG phải
// lớp bảo mật chính - HMAC vẫn là hàng rào thật quyết định.
void BLE_Key_SetOwnCarId(const char* carId);

// Dừng HẲN việc scan/kết nối lại - dùng khi Car báo lỗi VĨNH VIỄN (vd
// car_id không khớp). Khác disconnect thường (vẫn tự scan lại) - đây
// là trạng thái "chờ can thiệp thủ công" (kiểm tra lại provisioning),
// tiết kiệm pin/vô tuyến thay vì cứ thử lại vô ích.
void BLE_Key_HaltPermanently();

#endif