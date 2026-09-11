#include "can/isotp.h"
#include "can/twai_driver.h"
#include "shared/config.h"
#include <Arduino.h>
#include <string.h>

constexpr uint8_t ISOTP_PCI_SF = 0x0; // Single Frame
constexpr uint8_t ISOTP_PCI_FF = 0x1; // First Frame
constexpr uint8_t ISOTP_PCI_CF = 0x2; // Consecutive Frame
constexpr uint8_t ISOTP_PCI_FC = 0x3; // Flow Control

bool ISOTP_Send(uint32_t can_id, const uint8_t* data, size_t length, uint32_t timeoutMs)
{
    if (data == nullptr || length == 0)
        return false;

    uint32_t deadline = millis() + timeoutMs;

    // Single Frame: đủ chỗ trong 1 frame CAN (7 byte data, 1 byte PCI)
    if (length <= 7)
    {
        uint8_t frame[8] = {};
        frame[0] = (uint8_t)((ISOTP_PCI_SF << 4) | length);
        memcpy(frame + 1, data, length);
        return TWAI_Send(can_id, frame, (uint8_t)(length + 1));
    }

    if (length > 4095)
    {
        LOG_PRINTLN("[ISOTP ERROR] Payload quá lớn (>4095 byte)");
        return false;
    }

    // First Frame: 2 byte PCI+length, 6 byte data đầu
    uint8_t ffFrame[8] = {};
    ffFrame[0] = (uint8_t)((ISOTP_PCI_FF << 4) | ((length >> 8) & 0x0F));
    ffFrame[1] = (uint8_t)(length & 0xFF);
    memcpy(ffFrame + 2, data, 6);

    if (!TWAI_Send(can_id, ffFrame, 8))
    {
        LOG_PRINTLN("[ISOTP ERROR] Gửi First Frame thất bại");
        return false;
    }

    size_t sent = 6;

    // Chờ Flow Control (PCI=0x3) từ bên nhận trước khi gửi Consecutive Frame
    bool gotFC = false;
    while (millis() < deadline)
    {
        twai_message_t rxMsg;
        if (TWAI_Receive(rxMsg, pdMS_TO_TICKS(50)))
        {
            if (rxMsg.identifier == can_id && rxMsg.data_length_code >= 1 &&
                (rxMsg.data[0] >> 4) == ISOTP_PCI_FC)
            {
                gotFC = true;
                break;
            }
        }
    }

    if (!gotFC)
    {
        LOG_PRINTLN("[ISOTP ERROR] Không nhận được Flow Control");
        return false;
    }

    uint8_t seq = 1;
    while (sent < length)
    {
        if (millis() >= deadline)
        {
            LOG_PRINTLN("[ISOTP ERROR] Timeout khi gửi Consecutive Frame");
            return false;
        }

        size_t chunk = length - sent;
        if (chunk > 7)
            chunk = 7;

        uint8_t cfFrame[8] = {};
        cfFrame[0] = (uint8_t)((ISOTP_PCI_CF << 4) | (seq & 0x0F));
        memcpy(cfFrame + 1, data + sent, chunk);

        if (!TWAI_Send(can_id, cfFrame, (uint8_t)(chunk + 1)))
        {
            LOG_PRINTLN("[ISOTP ERROR] Gửi Consecutive Frame thất bại");
            return false;
        }

        sent += chunk;
        seq = (seq + 1) & 0x0F;
    }

    return true;
}

bool ISOTP_Receive(
    uint32_t can_id,
    uint8_t* outBuffer,
    size_t bufferCapacity,
    size_t& outLength,
    uint32_t timeoutMs)
{
    if (outBuffer == nullptr || bufferCapacity == 0)
        return false;

    uint32_t deadline = millis() + timeoutMs;

    // Chờ frame đầu tiên (SF hoặc FF)
    twai_message_t rxMsg;
    bool gotFirst = false;
    while (millis() < deadline)
    {
        if (TWAI_Receive(rxMsg, pdMS_TO_TICKS(50)))
        {
            if (rxMsg.identifier == can_id && rxMsg.data_length_code >= 1)
            {
                gotFirst = true;
                break;
            }
        }
    }

    if (!gotFirst)
        return false;

    uint8_t pciType = rxMsg.data[0] >> 4;

    if (pciType == ISOTP_PCI_SF)
    {
        size_t len = rxMsg.data[0] & 0x0F;
        if (len > bufferCapacity || (size_t)(len + 1) > (size_t)rxMsg.data_length_code)
        {
            LOG_PRINTLN("[ISOTP ERROR] Single Frame không hợp lệ");
            return false;
        }

        memcpy(outBuffer, rxMsg.data + 1, len);
        outLength = len;
        return true;
    }

    if (pciType != ISOTP_PCI_FF)
    {
        LOG_PRINTF("[ISOTP ERROR] Nhận frame không mong đợi (PCI=0x%X)\n", pciType);
        return false;
    }

    if (rxMsg.data_length_code < 8)
    {
        LOG_PRINTLN("[ISOTP ERROR] First Frame thiếu byte");
        return false;
    }

    size_t totalLen = (size_t)(((rxMsg.data[0] & 0x0F) << 8) | rxMsg.data[1]);
    if (totalLen > bufferCapacity)
    {
        LOG_PRINTLN("[ISOTP ERROR] Payload quá lớn so với buffer nhận");
        return false;
    }

    size_t received = 6;
    memcpy(outBuffer, rxMsg.data + 2, received);

    // Gửi Flow Control: Clear To Send, gửi hết 1 lần (block size=0, STmin=0)
    uint8_t fcFrame[3] = { (uint8_t)(ISOTP_PCI_FC << 4), 0x00, 0x00 };
    if (!TWAI_Send(can_id, fcFrame, 3))
    {
        LOG_PRINTLN("[ISOTP ERROR] Gửi Flow Control thất bại");
        return false;
    }

    uint8_t expectedSeq = 1;
    while (received < totalLen)
    {
        if (millis() >= deadline)
        {
            LOG_PRINTLN("[ISOTP ERROR] Timeout khi nhận Consecutive Frame");
            return false;
        }

        twai_message_t cfMsg;
        if (!TWAI_Receive(cfMsg, pdMS_TO_TICKS(50)))
            continue;

        if (cfMsg.identifier != can_id || cfMsg.data_length_code < 1)
            continue;

        uint8_t cfPci = cfMsg.data[0] >> 4;
        uint8_t cfSeq = cfMsg.data[0] & 0x0F;

        if (cfPci != ISOTP_PCI_CF)
        {
            LOG_PRINTF("[ISOTP ERROR] Mong đợi Consecutive Frame, nhận PCI=0x%X\n", cfPci);
            return false;
        }
        if (cfSeq != expectedSeq)
        {
            LOG_PRINTF("[ISOTP ERROR] Sai thứ tự Consecutive Frame (mong %u, nhận %u)\n",
                       expectedSeq, cfSeq);
            return false;
        }

        size_t chunk = totalLen - received;
        if (chunk > 7)
            chunk = 7;
        if (chunk > (size_t)(cfMsg.data_length_code - 1))
            chunk = (size_t)(cfMsg.data_length_code - 1);

        memcpy(outBuffer + received, cfMsg.data + 1, chunk);
        received += chunk;
        expectedSeq = (expectedSeq + 1) & 0x0F;
    }

    outLength = received;
    return true;
}