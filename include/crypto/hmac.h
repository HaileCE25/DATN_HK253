#pragma once
#include <Arduino.h>

void Crypto_Generate_HMAC(const uint8_t* nonce, uint8_t* out_token);
bool Crypto_Verify_HMAC(const uint8_t* received_token, const uint8_t* saved_nonce);