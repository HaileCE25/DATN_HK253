#pragma once
#include <Arduino.h>

// Đặt key_root động lúc runtime (dùng cho Car - nhận key từ Gateway qua
// UART/CAN). Nếu KHÔNG BAO GIỜ gọi hàm này (như Keyfob hiện tại), module
// tự dùng key mặc định hardcode trong hmac.cpp - giữ đúng hành vi "set
// cứng cho Keyfob" đã thống nhất.
// key_len phải <= 32, nếu không hợp lệ hàm bỏ qua, giữ nguyên key cũ.
void Crypto_SetKey(const uint8_t* key, size_t key_len);

void Crypto_Generate_HMAC(const uint8_t* nonce, uint8_t* out_token);
bool Crypto_Verify_HMAC(const uint8_t* received_token, const uint8_t* saved_nonce);

// Derive key phiên UWB từ nonce BLE (HKDF-SHA256).
// IKM  = key_root nội bộ (s_keyRoot trong hmac.cpp)
// Salt = nonce (16 byte, truyền vào qua salt + salt_len)
// Info = "uwb_sts_session" (cố định)
// Kết quả: out_len byte (thường 16) ghi vào out.
// Trả false nếu mbedtls_hkdf thất bại.
bool Crypto_HKDF(const uint8_t* salt, size_t salt_len,
                 uint8_t* out, size_t out_len);
