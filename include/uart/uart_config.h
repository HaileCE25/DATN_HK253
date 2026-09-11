#pragma once

#include <driver/gpio.h>
#include <stdint.h>

// -----------------------------------------------------------------------------
// UART Subsystem Configuration
// -----------------------------------------------------------------------------

#if defined(IS_CAR)

constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_4;
constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_5;

#elif defined(IS_GATEWAY)

// LƯU Ý QUAN TRỌNG: KHÔNG dùng GPIO43/44 - dù Serial debug đi qua
// USB-CDC gốc (không dùng GPIO43/44 ở tầng phần mềm), nhiều board
// ESP32-S3 DevKitC (bao gồm YoloUNO) có SẴN 1 chip cầu USB-UART vật lý
// (CP2102/CH340...) hàn cứng vào đúng GPIO43/44 để phục vụ cổng "COM"
// thứ 2 trên board. Nếu firmware tự chiếm GPIO43/44 làm UART1, nó xung
// đột trực tiếp với chip cầu đó, khiến nạp code qua cổng COM bị lỗi.
// Đã xác nhận qua thực nghiệm - đổi sang cặp chân khác, không liên
// quan tới UART0 mặc định.
constexpr gpio_num_t UART_TX_PIN = GPIO_NUM_17;
constexpr gpio_num_t UART_RX_PIN = GPIO_NUM_18;

#else

#error "uart_config.h included but neither IS_CAR nor IS_GATEWAY is defined"

#endif

constexpr uint32_t UART_BAUD_RATE = 115200;
constexpr int UART_PORT_NUM = 1;