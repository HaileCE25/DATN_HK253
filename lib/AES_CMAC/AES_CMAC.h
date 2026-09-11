#ifndef __AES_CMAC_H__
#define __AES_CMAC_H__

#include <Arduino.h>
#include <AES.h>
#include <BlockCipher.h>

// AES-CMAC (RFC 4493 / NIST SP 800-38B) generic over any AES key size.
//
// This is a fork of Obsttube/AES_CMAC (itself a fork of
// IndustrialShields/arduino-AES_CMAC), generalized to take a BlockCipher&
// instead of a hardcoded AESTiny128&. The CMAC algorithm/subkey derivation
// only depends on the AES block size (always 16 bytes for AES-128/192/256),
// not on the key length, so this works unmodified with AESTiny128,
// AESTiny256, AES128, AES192, AES256, etc. Key length is taken from
// cipher.keySize() instead of being hardcoded to 16.
class AES_CMAC {
	public:
		explicit AES_CMAC(BlockCipher& cipher);

	public:
		void generateMAC(uint8_t* mac, const uint8_t* key, const uint8_t* data, size_t dataLen);

	private:
		void shiftLeft(uint8_t* buff, uint8_t buffLen);
		void xor128(uint8_t* out, const uint8_t* a, const uint8_t* b);
		void padding(uint8_t* pad, const uint8_t* lastb, int len);

	private:
		BlockCipher& cipher;
		uint8_t X[16];
		uint8_t Y[16];
};

#endif // __AES_CMAC_H__
