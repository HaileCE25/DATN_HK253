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

// HKDF-SHA256 (RFC 5869) tự cài đặt.
// Lý do: libmbedcrypto.a của Arduino-ESP32 được build với CONFIG_MBEDTLS_HKDF_C
// tắt, nên mbedtls_hkdf() có prototype trong header nhưng KHÔNG có thân hàm
// -> undefined reference lúc link. mbedtls_md_hmac_* thì luôn có, nên ta dựng
// extract + expand từ HMAC-SHA256.
static bool hkdf_sha256(const uint8_t* salt, size_t salt_len,
                        const uint8_t* ikm,  size_t ikm_len,
                        const uint8_t* info, size_t info_len,
                        uint8_t* out, size_t out_len)
{
    const size_t HASH_LEN = 32;
    if (out_len > 255 * HASH_LEN)
        return false; // giới hạn của RFC 5869

    const mbedtls_md_info_t* md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md_info == nullptr)
        return false;

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    if (mbedtls_md_setup(&ctx, md_info, 1) != 0) {
        mbedtls_md_free(&ctx);
        return false;
    }

    bool ok = true;

    // --- Extract: PRK = HMAC(salt, IKM) ---
    uint8_t prk[32];
    const uint8_t zero_salt[32] = {0};
    if (salt == nullptr || salt_len == 0) {
        salt = zero_salt;
        salt_len = HASH_LEN;
    }
    ok = ok && mbedtls_md_hmac_starts(&ctx, salt, salt_len) == 0;
    ok = ok && mbedtls_md_hmac_update(&ctx, ikm, ikm_len) == 0;
    ok = ok && mbedtls_md_hmac_finish(&ctx, prk) == 0;

    // --- Expand: T(i) = HMAC(PRK, T(i-1) | info | i) ---
    uint8_t t[32];
    size_t t_len = 0;   // lần lặp đầu T(0) rỗng
    size_t done = 0;
    for (uint8_t counter = 1; ok && done < out_len; counter++) {
        ok = ok && mbedtls_md_hmac_starts(&ctx, prk, HASH_LEN) == 0;
        if (t_len)
            ok = ok && mbedtls_md_hmac_update(&ctx, t, t_len) == 0;
        if (info != nullptr && info_len)
            ok = ok && mbedtls_md_hmac_update(&ctx, info, info_len) == 0;
        ok = ok && mbedtls_md_hmac_update(&ctx, &counter, 1) == 0;
        ok = ok && mbedtls_md_hmac_finish(&ctx, t) == 0;
        if (!ok) break;

        t_len = HASH_LEN;
        size_t chunk = (out_len - done < HASH_LEN) ? (out_len - done) : HASH_LEN;
        memcpy(out + done, t, chunk);
        done += chunk;
    }

    mbedtls_md_free(&ctx);
    memset(prk, 0, sizeof(prk));
    memset(t, 0, sizeof(t));
    return ok;
}

bool Crypto_HKDF(const uint8_t* salt, size_t salt_len,
                 uint8_t* out, size_t out_len)
{
    // info cố định cho UWB STS session key
    static const uint8_t kInfo[] = "uwb_sts_session";
    const size_t kInfoLen = sizeof(kInfo) - 1; // bỏ null terminator

    return hkdf_sha256(salt, salt_len,          // salt = nonce BLE (16 byte)
                       s_keyRoot, s_keyLen,    // IKM  = key_root
                       kInfo, kInfoLen,        // info = "uwb_sts_session"
                       out, out_len);          // OKM output
}
