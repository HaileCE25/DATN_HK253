#include "uart/uart_transport.h"
#include "uart/uart_config.h"
#include "shared/config.h"
#include <Arduino.h>
#include <HardwareSerial.h>

static HardwareSerial UartPort(UART_PORT_NUM);
static bool s_initialized = false;

bool UART_Init()
{
    if (s_initialized)
        return true;

    UartPort.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    s_initialized = true;
    LOG_PRINTF("[UART] Initialized (TX=%d RX=%d baud=%lu)\n",
               (int)UART_TX_PIN, (int)UART_RX_PIN, (unsigned long)UART_BAUD_RATE);
    return true;
}

bool UART_SendFrame(const UartFrame& frame)
{
    if (!s_initialized)
        return false;

    if (frame.length > UART_MAX_PAYLOAD)
        return false;

    UartPort.write(frame.type);
    UartPort.write(frame.length);
    if (frame.length > 0)
    {
        UartPort.write(frame.data, frame.length);
    }

    return true;
}

// Đọc chính xác 1 byte, blocking tối đa timeoutMs (tính từ deadline
// chung của cả frame, không phải riêng từng byte).
static bool ReadByteWithDeadline(uint8_t& outByte, uint32_t deadline)
{
    while (millis() < deadline)
    {
        if (UartPort.available())
        {
            outByte = (uint8_t)UartPort.read();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool UART_ReceiveFrame(UartFrame& outFrame, uint32_t timeoutMs)
{
    if (!s_initialized)
        return false;

    uint32_t deadline = millis() + timeoutMs;

    uint8_t type, length;
    if (!ReadByteWithDeadline(type, deadline))
        return false;
    if (!ReadByteWithDeadline(length, deadline))
        return false;

    if (length > UART_MAX_PAYLOAD)
    {
        LOG_PRINTF("[UART ERROR] Invalid frame length (%u)\n", length);
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        if (!ReadByteWithDeadline(outFrame.data[i], deadline))
            return false;
    }

    outFrame.type = type;
    outFrame.length = length;
    return true;
}