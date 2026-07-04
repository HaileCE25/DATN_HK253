#include "crypto/hmac.h"
#include "mbedtls/md.h"

static const uint8_t KEY_ROOT[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

void Crypto_Generate_HMAC(const uint8_t* nonce, uint8_t* out_token)
{
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    
    mbedtls_md_hmac_starts(&ctx, KEY_ROOT, sizeof(KEY_ROOT));
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