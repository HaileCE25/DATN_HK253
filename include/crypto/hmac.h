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