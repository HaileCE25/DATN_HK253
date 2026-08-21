#pragma once

#include <driver/gpio.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// CAN Subsystem Configuration
// -----------------------------------------------------------------------------

#if defined(IS_CAR)

constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_17;
constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_18;

#elif defined(IS_GATEWAY)

constexpr gpio_num_t TWAI_TX_PIN = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_PIN = GPIO_NUM_5;

#else

#error "can_config.h included but neither IS_CAR nor IS_GATEWAY is defined"

#endif

constexpr uint32_t TWAI_BITRATE = 500000;