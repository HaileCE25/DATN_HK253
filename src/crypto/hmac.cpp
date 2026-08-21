#include "crypto/hmac.h"
#include "mbedtls/md.h"

// Key mặc định (fallback) - Keyfob dùng nguyên giá trị này vì không bao
// giờ gọi Crypto_SetKey() (quyết định "gán cứng cho Keyfob", đồng bộ thủ
// công với Firebase qua Admin Tool). Car sẽ ghi đè bằng Crypto_SetKey()
// sau khi nhận key_root từ Gateway.
static uint8_t s_keyRoot[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};
static size_t s_keyLen = 32;

void Crypto_SetKey(const uint8_t* key, size_t key_len)
{
    if (key == nullptr || key_len == 0 || key_len > sizeof(s_keyRoot))
        return; // input không hợp lệ - giữ nguyên key hiện tại

    memcpy(s_keyRoot, key, key_len);
    s_keyLen = key_len;

    // Xoá phần dư nếu key mới ngắn hơn key cũ, tránh dùng nhầm byte rác
    // còn sót từ lần set trước.
    if (key_len < sizeof(s_keyRoot))
    {
        memset(s_keyRoot + key_len, 0, sizeof(s_keyRoot) - key_len);
    }
}

void Crypto_Generate_HMAC(const uint8_t* nonce, uint8_t* out_token)
{
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    
    mbedtls_md_hmac_starts(&ctx, s_keyRoot, s_keyLen);
    mbedtls_md_hmac_update(&ctx, nonce, 16);
    mbedtls_md_hmac_finish(&ctx, out_token);
    
    mbedtls_md_free(&ctx);
}

bool Crypto_Verify_HMAC(const uint8_t* received_token, const uint8_t* saved_nonce) 
{
    uint8_t expected_token[32]; 
    
    Crypto_Generate_HMAC(saved_nonce, expected_token);

    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) {
        diff |= (received_token[i] ^ expected_token[i]);
    }
    
    return (diff == 0);
}