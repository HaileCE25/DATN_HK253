#pragma once

// -----------------------------------------------------------------------------
// RFID/NFC Subsystem Configuration (Car only)
// -----------------------------------------------------------------------------
// Tránh GPIO4/5 (đang dùng cho CAN) và GPIO43/44 (Serial debug qua
// UART0 vật lý - Car dùng chip cầu, không phải USB-CDC gốc như Gateway).

constexpr int RFID_SCK_PIN  = 47;
constexpr int RFID_MISO_PIN = 48;
constexpr int RFID_MOSI_PIN = 10;
constexpr int RFID_SS_PIN   = 9;
constexpr int RFID_RST_PIN  = 8;