#ifndef OS_QUEUE_H
#define OS_QUEUE_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "protocol/packet.h"
#include "shared/config.h"

extern QueueHandle_t bleRxQueue;
extern QueueHandle_t bleTxQueue;

bool Queue_Init();
void Queue_Deinit();

#endif