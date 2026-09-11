#include "nfc/rfid_hal.h"
#include "nfc/rfid_config.h"
#include <MFRC522.h>
#include <SPI.h>
#include "shared/config.h"

static MFRC522* s_mfrc522 = nullptr;
static bool s_initialized = false;

static MFRC522::MIFARE_Key s_defaultKey = {
    .keyByte = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

bool NFC_Init()
{
    if (s_initialized)
        return true;

    SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);

    s_mfrc522 = new MFRC522(RFID_SS_PIN, RFID_RST_PIN);
    s_mfrc522->PCD_Init();

    s_initialized = true;
    LOG_PRINTLN("[NFC] RC522 initialized");
    return true;
}

bool NFC_TryReadCard(uint8_t* outUid, uint8_t& outUidLen, uint8_t maxUidLen)
{
    if (!s_initialized)
        return false;

    if (!s_mfrc522->PICC_IsNewCardPresent())
        return false;

    if (!s_mfrc522->PICC_ReadCardSerial())
        return false;

    uint8_t len = s_mfrc522->uid.size;
    if (len > maxUidLen)
        len = maxUidLen;

    memcpy(outUid, s_mfrc522->uid.uidByte, len);
    outUidLen = len;

    // KHONG goi PICC_HaltA() o day nua - neu nguoi goi can ghi/doc them
    // block du lieu ngay sau do, phien lam viec voi the phai con mo.
    // Goi NFC_EndSession() sau khi xong moi thao tac.
    return true;
}

bool NFC_ReadBlock(uint8_t blockAddr, uint8_t* outData16Bytes)
{
    if (!s_initialized)
        return false;

    MFRC522::StatusCode status = s_mfrc522->PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &s_defaultKey, &(s_mfrc522->uid));

    if (status != MFRC522::STATUS_OK)
    {
        LOG_PRINTF("[NFC ERROR] Auth thất bại (block %u): %s\n",
                   blockAddr, s_mfrc522->GetStatusCodeName(status));
        return false;
    }

    byte bufferSize = 18;
    status = s_mfrc522->MIFARE_Read(blockAddr, outData16Bytes, &bufferSize);

    if (status != MFRC522::STATUS_OK)
    {
        LOG_PRINTF("[NFC ERROR] Đọc block %u thất bại: %s\n",
                   blockAddr, s_mfrc522->GetStatusCodeName(status));
        return false;
    }

    return true;
}

bool NFC_WriteBlock(uint8_t blockAddr, const uint8_t* data16Bytes)
{
    if (!s_initialized)
        return false;

    MFRC522::StatusCode status = s_mfrc522->PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockAddr, &s_defaultKey, &(s_mfrc522->uid));

    if (status != MFRC522::STATUS_OK)
    {
        LOG_PRINTF("[NFC ERROR] Auth thất bại (block %u): %s\n",
                   blockAddr, s_mfrc522->GetStatusCodeName(status));
        return false;
    }

    status = s_mfrc522->MIFARE_Write(blockAddr, (byte*)data16Bytes, 16);

    if (status != MFRC522::STATUS_OK)
    {
        LOG_PRINTF("[NFC ERROR] Ghi block %u thất bại: %s\n",
                   blockAddr, s_mfrc522->GetStatusCodeName(status));
        return false;
    }

    return true;
}

// Sector 1 (block 4,5,6 = data, block 7 = trailer - khong dung toi).
constexpr uint8_t NFC_DATA_BLOCK_START = 4;
constexpr uint8_t NFC_DATA_NUM_BLOCKS = 3;

bool NFC_WriteData(const uint8_t* data, size_t dataLen)
{
    if (dataLen > NFC_DATA_MAX_LEN)
    {
        LOG_PRINTF("[NFC ERROR] Dữ liệu quá dài (%u byte, tối đa %u)\n",
                   (unsigned)dataLen, (unsigned)NFC_DATA_MAX_LEN);
        return false;
    }

    uint8_t buffer[NFC_DATA_MAX_LEN] = {}; // phan con lai tu dong la 0x00
    memcpy(buffer, data, dataLen);

    for (uint8_t i = 0; i < NFC_DATA_NUM_BLOCKS; i++)
    {
        if (!NFC_WriteBlock(NFC_DATA_BLOCK_START + i, buffer + (i * 16)))
        {
            return false;
        }
    }

    return true;
}

bool NFC_ReadData(uint8_t* outData, size_t maxLen, size_t& outLen)
{
    uint8_t buffer[NFC_DATA_MAX_LEN] = {};

    for (uint8_t i = 0; i < NFC_DATA_NUM_BLOCKS; i++)
    {
        uint8_t block[18] = {};
        if (!NFC_ReadBlock(NFC_DATA_BLOCK_START + i, block))
        {
            return false;
        }
        memcpy(buffer + (i * 16), block, 16);
    }

    // Tim null-terminator (neu du lieu duoc ghi la text ket thuc som
    // hon 48 byte) - neu khong co, dung het NFC_DATA_MAX_LEN.
    size_t len = strnlen((char*)buffer, NFC_DATA_MAX_LEN);
    if (len > maxLen)
        len = maxLen;

    memcpy(outData, buffer, len);
    outLen = len;
    return true;
}

void NFC_EndSession()
{
    if (!s_initialized)
        return;

    s_mfrc522->PICC_HaltA();
    s_mfrc522->PCD_StopCrypto1();
}