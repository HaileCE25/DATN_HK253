#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// RFID/NFC HAL (RC522) - tầng phần cứng thuần, không biết gì về whitelist
// hay nghiệp vụ mở khóa. Giống nguyên tắc uwb_hal.h: chỉ đọc UID thẻ.
// -----------------------------------------------------------------------------

bool NFC_Init();

// Non-blocking: trả true nếu vừa quẹt được 1 thẻ mới, điền UID vào
// outUid (tối đa maxUidLen byte), outUidLen là độ dài UID thật (4 hoặc
// 7 byte tùy loại thẻ Mifare). Trả false nếu không có thẻ nào đang quẹt.
bool NFC_TryReadCard(uint8_t* outUid, uint8_t& outUidLen, uint8_t maxUidLen);

// Đọc/ghi 1 block dữ liệu (16 byte) trên thẻ Mifare Classic đang quẹt.
// blockAddr: số block (0-63 cho thẻ 1K) - TRÁNH block 0 (dữ liệu nhà
// sản xuất) và các trailer block (block cuối mỗi sector, chứa key -
// ghi đè sẽ hỏng quyền truy cập sector đó). Dùng key mặc định xuất
// xưởng (0xFFFFFFFFFFFF) - đúng cho thẻ mới, chưa từng đổi key.
// Phải gọi ngay sau NFC_TryReadCard() trả true (thẻ vẫn đang trong
// vùng đọc), trước khi PICC_HaltA() được gọi.
bool NFC_WriteBlock(uint8_t blockAddr, const uint8_t* data16Bytes);
bool NFC_ReadBlock(uint8_t blockAddr, uint8_t* outData16Bytes);

// Ghi/đọc dữ liệu DÀI HƠN 16 byte - ghép 3 block liền trong cùng 1
// sector (sector 1: block 4,5,6 = 48 byte data, block 7 là trailer,
// KHÔNG đụng tới). Đủ chứa nguyên mã booking_id 20 ký tự của Firebase
// push key, hoặc chuỗi tùy ý tối đa 47 ký tự (dành 1 byte cuối làm
// null-terminator khi đọc lại).
constexpr size_t NFC_DATA_MAX_LEN = 48;

bool NFC_WriteData(const uint8_t* data, size_t dataLen);
bool NFC_ReadData(uint8_t* outData, size_t maxLen, size_t& outLen);

// Kết thúc phiên làm việc với thẻ (Halt + dừng mã hóa Crypto1) - PHẢI
// gọi sau khi xong đọc/ghi, nếu không thẻ vẫn "authenticated" và có
// thể gây lỗi ở lần đọc thẻ tiếp theo.
void NFC_EndSession();