#include "can/twai_driver.h"
#include "can/can_config.h"
#include "shared/config.h"
#include <Arduino.h>

static bool s_initialized = false;

bool TWAI_Init()
{
    if (s_initialized)
        return true;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    // Mặc định 5 frame. Gateway có thể kẹt 1-2 s trong lời gọi Firebase (tra
    // booking mỗi lần Car hỏi lại key); CarStatus đổ vào tới ~10 frame/s sẽ làm
    // tràn queue và rơi mất ActuatorCmd đến cùng lúc.
    g_config.rx_queue_len = 32;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        LOG_PRINTLN("[CAN ERROR] Driver install failed");
        return false;
    }

    if (twai_start() != ESP_OK)
    {
        LOG_PRINTLN("[CAN ERROR] Driver start failed");
        return false;
    }

    s_initialized = true;
    LOG_PRINTLN("[CAN] TWAI initialized");
    return true;
}

bool TWAI_Send(uint32_t id, const uint8_t* data, uint8_t dlc)
{
    if (!s_initialized)
        return false;

    if (dlc > 8)
        return false;

    if (data == nullptr && dlc > 0)
        return false;

    twai_message_t message = {};
    message.identifier = id;
    message.data_length_code = dlc;
    message.extd = 0;
    message.rtr = 0;

    for (uint8_t i = 0; i < dlc; i++)
    {
        message.data[i] = data[i];
    }

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK)
    {
        LOG_PRINTLN("[CAN ERROR] Transmit failed");
        return false;
    }

    return true;
}

bool TWAI_Receive(twai_message_t& message, TickType_t timeout)
{
    if (!s_initialized)
        return false;

    return twai_receive(&message, timeout) == ESP_OK;
}