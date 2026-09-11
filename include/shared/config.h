#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// -----------------------------------------------------------------------------
// Shared Config - hằng số dùng chung toàn hệ thống (BLE UUID, timeout...)
// -----------------------------------------------------------------------------

#define SERVICE_UUID                "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX      "1c4224ce-8d26-444a-9366-a4968848db7f"
#define CHARACTERISTIC_UUID_RX      "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define DEVICE_NAME_CAR             "ESP32_CAR_ECU"
#define DEVICE_NAME_KEYFOB          "ESP32_KEYFOB"

constexpr int MAX_PACKET_SIZE = 128;
constexpr uint32_t BLE_TIMEOUT_MS = 3000;
constexpr uint32_t BLE_COOLDOWN_MS = 3000;
constexpr int BLE_CONNECT_RSSI_THRESHOLD = -80;
constexpr int BLE_QUEUE_LENGTH = 10;

// -----------------------------------------------------------------------------
// THREAD-SAFE LOGGING
// -----------------------------------------------------------------------------
// QUAN TRỌNG: nhiều task FreeRTOS (TaskBLE, TaskLogic, TaskKeyRequest,
// TaskNFC, TaskNFCProvisioning...) đều gọi Serial.print đồng thời -
// KHÔNG có mutex, các byte có thể bị CHEN NGANG lẫn nhau ở tầng UART,
// làm hỏng nội dung log (vd chuỗi "SUCCESS" bị cắt vụn/lẫn ký tự lạ,
// khiến Web Tool không nhận diện được phản hồi đúng - đã xác nhận qua
// thực nghiệm). Dùng static local mutex (Meyer's singleton, tự khởi
// tạo an toàn lần gọi đầu tiên, không cần thêm dòng nào trong setup()
// của từng board) để đảm bảo mỗi lần in log là 1 khối nguyên vẹn,
// không bị task khác chen vào giữa.

inline SemaphoreHandle_t GetSerialMutex()
{
    static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
    return mutex;
}

#define LOG_PRINTLN(x) do { \
    SemaphoreHandle_t _m = GetSerialMutex(); \
    xSemaphoreTake(_m, portMAX_DELAY); \
    Serial.println(x); \
    xSemaphoreGive(_m); \
} while (0)

#define LOG_PRINTF(...) do { \
    SemaphoreHandle_t _m = GetSerialMutex(); \
    xSemaphoreTake(_m, portMAX_DELAY); \
    Serial.printf(__VA_ARGS__); \
    xSemaphoreGive(_m); \
} while (0)

