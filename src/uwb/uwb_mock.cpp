#include "uwb/uwb_hal.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "shared/config.h"

// -----------------------------------------------------------------------------
// UWB Mock Implementation
// -----------------------------------------------------------------------------
// Chỉ compile khi build với -D UWB_USE_MOCK=1 (xem platformio.ini).
// Mô phỏng ranging bằng cách dao động ngẫu nhiên quanh 1.5m, đủ để test
// logic unlock-zone (S3/S4 trong state machine) mà không cần DW3000EVB.
// Khi có phần cứng thật, file này bị loại khỏi build (build_src_filter),
// thay bằng uwb_dw3000.cpp - main.cpp không cần đổi gì.

static bool s_initialized = false;
static bool s_ranging = false;
static float s_lastDistance = 0.0f;
static bool s_hasDistance = false;
static TaskHandle_t s_rangingTaskHandle = nullptr;

static void RangingMockTask(void* pvParameters)
{
    for (;;)
    {
        if (s_ranging)
        {
            // Dao động giả lập quanh 1.5m, +/- 0.3m, để test logic
            // approach/unlock-zone với dữ liệu "động" thay vì hằng số.
            float noise = (float)(esp_random() % 600) / 1000.0f - 0.3f;
            s_lastDistance = 1.5f + noise;
            s_hasDistance = true;

            LOG_PRINTF("[UWB MOCK] distance=%.2f m\n", s_lastDistance);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // giả lập ~5Hz ranging rate
    }
}

bool UWB_Init()
{
    if (s_initialized)
        return true;

    LOG_PRINTLN("[UWB MOCK] Init (no real hardware)");
    s_initialized = true;
    return true;
}

bool UWB_StartRanging()
{
    if (!s_initialized)
        return false;

    if (s_ranging)
        return true; // idempotent

    s_ranging = true;
    s_hasDistance = false;

    if (s_rangingTaskHandle == nullptr)
    {
        // Tăng stack từ 2048 -> 4096: 2048 gây Stack Canary Overflow
        // (crash "Stack canary watchpoint triggered (UWBMock)") vì
        // LOG_PRINTF dùng %.2f - định dạng số thực tốn nhiều stack hơn
        // số nguyên trên newlib/ESP32. 4096 khớp với các task khác
        // trong dự án (TaskBLE, TaskLogic), đủ dư dả.
        xTaskCreatePinnedToCore(
            RangingMockTask, "UWBMock", 4096, nullptr, 1, &s_rangingTaskHandle, 1);
    }

    LOG_PRINTLN("[UWB MOCK] Ranging started");
    return true;
}

bool UWB_StopRanging()
{
    s_ranging = false;
    s_hasDistance = false;
    LOG_PRINTLN("[UWB MOCK] Ranging stopped");
    return true;
}

bool UWB_GetLastDistance(float& outMeters)
{
    if (!s_hasDistance)
        return false;

    outMeters = s_lastDistance;
    return true;
}