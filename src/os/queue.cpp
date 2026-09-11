#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "os/queue.h"

QueueHandle_t bleRxQueue = NULL;
QueueHandle_t bleTxQueue = NULL;

bool Queue_Init()
{
    bleRxQueue = xQueueCreate(
        BLE_QUEUE_LENGTH,
        sizeof(Packet));

    bleTxQueue = xQueueCreate(
        BLE_QUEUE_LENGTH,
        sizeof(Packet));

    if (bleRxQueue == nullptr)
    {
        return false;
    }

    if (bleTxQueue == nullptr)
    {
        vQueueDelete(bleRxQueue);
        bleRxQueue = nullptr;
        return false;
    }

    return true;
}

void Queue_Deinit()
{
    if (bleRxQueue != nullptr)
    {
        vQueueDelete(bleRxQueue);
        bleRxQueue = nullptr;
    }

    if (bleTxQueue != nullptr)
    {
        vQueueDelete(bleTxQueue);
        bleTxQueue = nullptr;
    }
}