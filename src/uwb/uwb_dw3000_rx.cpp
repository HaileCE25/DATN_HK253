/*
 * uwb_dw3000.cpp — hiện thực THẬT của include/uwb/uwb_hal.h trên DW3000 (DS-TWR).
 *
 * Board này (ECU Access, env "car") đóng vai RX/responder — con TÍNH ra khoảng
 * cách. Build khi KHÔNG định nghĩa UWB_USE_MOCK (xem platformio.ini [env:car]);
 * uwb_mock.cpp bị loại khỏi build_src_filter.
 *
 * ── Cấu trúc file ────────────────────────────────────────────────────────────
 *  Phần A: FSM ranging — COPY NGUYÊN VĂN từ reference/uwb_hal.cpp. KHÔNG sửa
 *          thuật toán DS-TWR / timing / STS / công thức tính khoảng cách.
 *
 *  Phần B: wrapper task — theo đúng mẫu trong uwb_mock.cpp:
 *          task loop cập nhật biến static s_lastDistance, cờ s_ranging bật/tắt,
 *          UWB_GetLastDistance() chỉ ĐỌC. ĐƠN VỊ: mét (reference trả cm -> /100).
 *          Thêm UWB_SetSessionKey() để bind K_session vào STS trước ranging.
 */

#include "uwb/uwb_hal.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "shared/config.h"
#include "DW3000_rx.h"

// Log mức HAL (khoảng cách, lỗi từng chu kỳ) — chỉ in NGOÀI cửa sổ timing
// DS-TWR, có mutex (LOG_PRINTF). Khác UWB_DEBUG (in bên trong FSM/driver).
#ifndef UWB_DW3000_DEBUG
#define UWB_DW3000_DEBUG 1
#endif

#if UWB_DW3000_DEBUG
#define UWB_LOG_PRINTF(...)  LOG_PRINTF(__VA_ARGS__)
#define UWB_LOG_PRINTLN(x)   LOG_PRINTLN(x)
#else
#define UWB_LOG_PRINTF(...)  do {} while (0)
#define UWB_LOG_PRINTLN(x)   do {} while (0)
#endif

// =============================================================================
//  PHẦN A — FSM ranging DS-TWR (copy từ reference/uwb_hal.cpp, KHÔNG sửa logic)
// =============================================================================

// Số chu kỳ đo mỗi lần lọc median (giữ nguyên reference/uwb_hal.h).
#ifndef UWB_MEASURE_SAMPLES
#define UWB_MEASURE_SAMPLES 5
#endif

// Số lần PREPOLL được phép tự lặp lại (mỗi lần = 1 cửa sổ timeout 5s chờ TX)
// trước khi 1 lần đo đơn bị coi là thất bại. Không đổi 5s, chỉ chặn số lần lặp.
#ifndef UWB_PREPOLL_MAX_RETRY
#define UWB_PREPOLL_MAX_RETRY 1
#endif

// Số lần hardReset được phép thử lại khi chờ chip vào IDLE trong rangingCoreInit()
// trước khi bỏ cuộc. Không đổi các mốc thời gian bên trong (100ms/lần check,
// hardReset sau 50 lần, delay 500ms) — chỉ chặn tổng số lần.
#ifndef UWB_INIT_MAX_HARDRESET_RETRY
#define UWB_INIT_MAX_HARDRESET_RETRY 5
#endif

// Timeout phần mềm (chặn trên) cho các vòng chờ TRONG cửa sổ DS-TWR
// (POLL/RESPOND/FINAL/FINALDATA). Các gói này đến cách nhau cỡ 4–12 ms, và
// FINAL/FINALDATA còn có RX timeout của chip (10 ms / 5 ms) -> 100 ms là dư.
// Các vòng này busy-wait (không vTaskDelay, giống bản standalone) để car bật
// lại RX kịp; giữ chặn trên nhỏ để lỡ gói không chiếm core 1 tới 2 s.
#ifndef UWB_CRITICAL_WAIT_MS
#define UWB_CRITICAL_WAIT_MS 100
#endif

// Chẩn đoán chu kỳ đo gần nhất bị hỏng — chỉ GHI trong FSM, IN ra sau khi
// chu kỳ kết thúc (xem rangingMedianCm) để không chen vào cửa sổ timing.
static uint8_t  s_failState    = 0;
static uint32_t s_failRxStatus = 0;
static uint32_t s_failTxStatus = 0;

static const char* stateName(uint8_t st)
{
    switch (st)
    {
    case PREPOLL:   return "PREPOLL";
    case POLL:      return "POLL";
    case RESPOND:   return "RESPOND";
    case FINAL:     return "FINAL";
    case FINALDATA: return "FINALDATA";
    case IDLE:      return "IDLE";
    default:        return "?";
    }
}

// ---- rangingCoreInit() ---------------------------------------------------------

static bool waitForIdleOrHardReset(uint8_t maxHardResets)
{
    uint8_t hardResetCount = 0;
    int idle_retry = 0;
    while (!DW3000.checkForIDLE())
    {
        idle_retry++;
        delay(100);
        if (idle_retry > 50)
        {
            if (UWB_DEBUG) Serial.println("[ERROR] IDLE FAILED - retrying hard reset");
            hardResetCount++;
            if (hardResetCount > maxHardResets)
            {
                return false;
            }
            DW3000.hardReset();
            delay(500);
            idle_retry = 0;
        }
    }
    return true;
}

static bool rangingCoreInit()
{
    DW3000.begin();       // Init SPI
    DW3000.hardReset();   // hard reset in case that the chip wasn't disconnected from power
    delay(200);            // Wait for DW3000 chip to wake up
    if (UWB_DEBUG) Serial.println("debug1");

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        if (UWB_DEBUG) Serial.println("[ERROR] rangingCoreInit: IDLE1 failed after max hard-reset retries");
        return false;
    }
    if (UWB_DEBUG) Serial.println("debug2 - IDLE1 OK");

    DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
    delay(200);          // Wait for DW3000 chip to wake up

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        if (UWB_DEBUG) Serial.println("[ERROR] rangingCoreInit: IDLE2 failed after max hard-reset retries");
        return false;
    }
    if (UWB_DEBUG) Serial.println("debug3 - IDLE2 OK");

    DW3000.init(); // Initialize chip (write default values, calibration, etc.)
    DW3000.write(0x0e, 0x16, 0x9b, 1);
    DW3000.configureAsTX();          // Configure basic settings for frame transmitting
    DW3000.setTXAntennaDelay(16385); // set default antenna delay

    DW3000.InitCrypto();
    if (UWB_DEBUG) DW3000.printFullConfig();
    delay(2000);
    DW3000.setupRFport(PORTRF_1);
    if (UWB_DEBUG) Serial.println("[INFO] Setup complete - RX ready, waiting for TX...");
    return true;
}

// ---- measure_once_cm(): 1 chu kỳ ranging DS-TWR đầy đủ ---------------------

static float measure_once_cm()
{
    uint8_t state = PREPOLL;
    uint8_t prepoll_retry_count = 0;

    uint32_t exact_tx_timestamp;
    long long rx_ts;
    uint32_t time_start = 0;
    uint32_t time_end;
    uint32_t time_diff;
    uint32_t rx_status = 0;
    uint32_t tx_status = 0;
    uint8_t buffer_prepoll[127];
    uint8_t buffer_finalData[127];
    uint8_t last_state = PREPOLL; // state đang chạy khi rơi vào nhánh lỗi (100)

    while (true)
    {
        DW3000.clearSystemStatus();
        if (state != 100) last_state = state;
        switch (state)
        {
        case PREPOLL:
        {
            DW3000.DisableTimeout();
            if (UWB_DEBUG) Serial.println("PREPOLL");

            memset(buffer_prepoll, 0xFF, sizeof(buffer_prepoll));
            DW3000.standardRX(); // Send command to DW3000 to start the reception of frames
            {
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.receivedFrameSucc_noCIA()))
                {
                    if (millis() - t0 > 5000) { rx_status = 0; break; } // 5s timeout
                    vTaskDelay(pdMS_TO_TICKS(1));
                };
            }
            if ((rx_status == 1) || (rx_status == 2))
            {
                DW3000.ReceivePrepoll();
                DW3000.read_payload(0x12, 0x00, 46, buffer_prepoll);
                DW3000.PrepollData(buffer_prepoll);
                DW3000.Generate_KtURSK();
                DW3000.Generate_dURSK();
                DW3000.WriteRespondSTS();
                DW3000.SwitchPackage(SP3);
                DW3000.WritePollSTS_IV();
                DW3000.write(0x02, 0x04, 1, 1);
                state = POLL;
            }
            else if (rx_status == 0)
            {
                if (UWB_DEBUG) Serial.println("[WARN] PREPOLL timeout, retrying...");
                DW3000.putIdle();
                prepoll_retry_count++;
                if (prepoll_retry_count > UWB_PREPOLL_MAX_RETRY)
                {
                    s_failState = PREPOLL;
                    s_failRxStatus = 0;
                    s_failTxStatus = 0;
                    return -1.0f;
                }
                state = PREPOLL;
            }
            else
            {
                state = 100;
            }

            break;
        }
        case POLL:
        {
            if (UWB_DEBUG) Serial.println("POLL");
            time_diff = 0;
            DW3000.standardRX();
            {
                // Busy-wait: POLL tới ~4 ms sau PREPOLL, không nhường tick ở đây.
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.receivedFrameSucc_test()))
                {
                    if (millis() - t0 > UWB_CRITICAL_WAIT_MS) { rx_status = 0; break; }
                };
            }
            if ((rx_status == 1))
            {
                rx_ts = DW3000.readRXTimestamp_Poll();

                DW3000.Receivepoll();

                time_end = millis();
                time_diff = time_end - time_start;
                exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY_12MS) >> 8;
                DW3000.WriteRespondSTS_IV();
                DW3000.write(0x02, 0x04, 1, 1);
                DW3000.writeTXDelay(exact_tx_timestamp);

                DW3000.delayedTX();
                state = RESPOND;
            }
            else
            {
                state = 100;
            }

            break;
        }
        case RESPOND:
        {
            if (UWB_DEBUG) Serial.println("RES");

            {
                // Busy-wait: keyfob phát FINAL NGAY khi nhận RESPOND (không delayed TX)
                // -> car phải bật RX cho FINAL trong vài ms sau khi RESPOND phát xong.
                uint32_t t0 = millis();
                while (!(tx_status = DW3000.sentFrameSucc()))
                {
                    if (millis() - t0 > UWB_CRITICAL_WAIT_MS) { tx_status = 0; break; }
                };
            }
            if (!tx_status) { state = 100; break; }
            DW3000.SendRespond();
            DW3000.putIdle();
            {
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.checkForIDLE()))
                {
                    if (millis() - t0 > UWB_CRITICAL_WAIT_MS) { rx_status = 0; break; }
                }
            }
            if (!rx_status) { state = 100; break; }
            DW3000.EnableTimeout();
            DW3000.WriteTimeOutPeriod(10000);
            rx_ts = DW3000.readTXTimestamp();
            DW3000.SwitchPackage(SP3);
            DW3000.WriteFinalSTS_IV();
            DW3000.write(0x02, 0x04, 1, 1);
            DW3000.standardRX();
            time_start = millis();
            state = FINAL;
            break;
        }
        case FINAL:
        {
            if (UWB_DEBUG) Serial.println("FINAL");

            {
                // Busy-wait: FINALDATA tới NGAY sau FINAL -> phải bật lại RX tức thì.
                uint32_t t0 = millis();
                while ((!(rx_status = DW3000.receivedFrameSucc_test())))
                {
                    if (millis() - t0 > UWB_CRITICAL_WAIT_MS) { rx_status = 0; break; }
                };
            }
            if (rx_status == 1)
            {
                if (UWB_DEBUG)
                {
                    uint32_t temp = DW3000.read(0x0C, 0x0C) & 0xFF800000;
                    Serial.print("0c0c : ");
                    Serial.println(temp, HEX);
                    temp = DW3000.read(0x0C, 0x00) & 0xFF800000;
                    Serial.print("0c00 : ");
                    Serial.println(temp, HEX);
                }

                rx_ts = DW3000.readRXTimestamp_Final();
                DW3000.putIdle();
                DW3000.SwitchPackage(SP0);
                DW3000.WriteTimeOutPeriod(5000);
                DW3000.standardRX();
                state = FINALDATA;
            }
            else
            {
                if (UWB_DEBUG) { Serial.print("FINAL failed, rx_status="); Serial.println(rx_status); }
                state = 100;
            }
            break;
        }
        case FINALDATA:
        {
            if (UWB_DEBUG) Serial.println("FINALDATA");

            {
                uint32_t t0 = millis();
                while ((!(rx_status = DW3000.receivedFrameSucc_noCIA())))
                {
                    if (millis() - t0 > UWB_CRITICAL_WAIT_MS) { rx_status = 0; break; }
                };
            }
            if (UWB_DEBUG)
            {
                Serial.print("FINALDATA rx_status=");
                Serial.print(rx_status);
                uint32_t ss = DW3000.read(0x00, 0x44);
                Serial.print(" sys=0x");
                Serial.println(ss, HEX);
            }

            if (rx_status == 1)
            {
                DW3000.Receivefinaldata();
                memset(buffer_finalData, 0xFF, sizeof(buffer_finalData));
                DW3000.read_payload(0x12, 0x00, 127, buffer_finalData);
                time_start = millis();
                state = IDLE;
            }
            else
            {
                state = 100;
            }

            break;
        }
        case IDLE:
        {
            DW3000.putIdle();
            DW3000.SwitchPackage(SP0);
            DW3000.setTXFrame(0x00);
            DW3000.setFrameLength(0);
            if (UWB_DEBUG) { Serial.print("IDLE: finalData[19]=0x"); Serial.println(buffer_finalData[19], HEX); }
            DW3000.FinalData(buffer_finalData);
            float distance_cm = DW3000.getLastDistance();
            DW3000.Cleartimestamp();

            return distance_cm;
        }
        default:
        {
            s_failState = last_state;
            s_failRxStatus = rx_status;
            s_failTxStatus = tx_status;
            DW3000.putIdle();
            DW3000.Cleartimestamp();
            DW3000.SwitchPackage(SP0);
            DW3000.setTXFrame(0x00);
            DW3000.setFrameLength(0);

            return -1.0f;
        }
        }
    }
}

// ---- rangingMedianCm(): lọc median trên N chu kỳ đo ------------------------

static int compareFloatAsc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float rangingMedianCm()
{
    float samples[UWB_MEASURE_SAMPLES];
    int valid = 0;

    for (int i = 0; i < UWB_MEASURE_SAMPLES; i++)
    {
        float d = measure_once_cm();
        if (d >= 0.0f)
        {
            samples[valid++] = d;
        }
        else
        {
            // In SAU khi chu kỳ đã kết thúc -> không ảnh hưởng timing DS-TWR.
            UWB_LOG_PRINTF("[UWB DW3000] sample %d fail @%s rx_status=%u tx_status=%u\n",
                           i, stateName(s_failState),
                           (unsigned)s_failRxStatus, (unsigned)s_failTxStatus);
        }
    }

    if (valid * 2 <= UWB_MEASURE_SAMPLES)
    {
        return -1.0f;
    }

    qsort(samples, valid, sizeof(float), compareFloatAsc);
    return samples[valid / 2];
}

// =============================================================================
//  PHẦN B — wrapper task, hiện thực include/uwb/uwb_hal.h
// =============================================================================

static bool s_initialized = false;
static bool s_ranging = false;
static float s_lastDistance = 0.0f; // ĐƠN VỊ: MÉT
static bool s_hasDistance = false;
static TaskHandle_t s_rangingTaskHandle = nullptr;

// K_session lưu trữ, nạp vào DW3000 trước khi ranging
static uint8_t s_sessionKey[16] = {0};
static bool s_sessionKeySet = false;

static void RangingTask(void* pvParameters)
{
    for (;;)
    {
        if (s_ranging)
        {
            float cm = rangingMedianCm();
            if (cm >= 0.0f)
            {
                s_lastDistance = cm / 100.0f;
                s_hasDistance = true;
                UWB_LOG_PRINTF("[UWB DW3000] distance=%.2f m\n", s_lastDistance);
            }
            else
            {
                UWB_LOG_PRINTLN("[UWB DW3000] measurement failed (giu ket qua cu)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool UWB_Init()
{
    if (s_initialized)
        return true; // idempotent

    UWB_LOG_PRINTLN("[UWB DW3000] Init");
    if (!rangingCoreInit())
    {
        UWB_LOG_PRINTLN("[UWB DW3000] Init FAILED (chip khong vao IDLE)");
        return false;
    }

    s_initialized = true;
    UWB_LOG_PRINTLN("[UWB DW3000] Init OK");
    return true;
}

bool UWB_SetSessionKey(const uint8_t* key, size_t len)
{
    if (key == nullptr || len == 0 || len > 16)
        return false;

    // Lưu key vào buffer nội bộ
    memset(s_sessionKey, 0, sizeof(s_sessionKey));
    memcpy(s_sessionKey, key, len);
    s_sessionKeySet = true;

    // Tái khởi tạo STS crypto với K_session mới:
    // QUAN TRỌNG: phải ghi register TRƯỚC, rồi mới gọi InitCrypto().
    // InitCrypto() seed AES từ giá trị đang ở STS_KEY register tại thời điểm gọi.
    // Nếu gọi InitCrypto() trước → nó seed key cũ (default), ghi key mới sau vô nghĩa.

    // 1. Ghi 16 byte K_session vào STS_KEY registers (SET_1_2_REG = 0x18,
    //    4 word × 4 byte, offset 0x04/0x08/0x0C/0x10 theo DW3000 User Manual).
    uint32_t w0 = (uint32_t)s_sessionKey[0]  | ((uint32_t)s_sessionKey[1]  << 8)
                | ((uint32_t)s_sessionKey[2]  << 16) | ((uint32_t)s_sessionKey[3]  << 24);
    uint32_t w1 = (uint32_t)s_sessionKey[4]  | ((uint32_t)s_sessionKey[5]  << 8)
                | ((uint32_t)s_sessionKey[6]  << 16) | ((uint32_t)s_sessionKey[7]  << 24);
    uint32_t w2 = (uint32_t)s_sessionKey[8]  | ((uint32_t)s_sessionKey[9]  << 8)
                | ((uint32_t)s_sessionKey[10] << 16) | ((uint32_t)s_sessionKey[11] << 24);
    uint32_t w3 = (uint32_t)s_sessionKey[12] | ((uint32_t)s_sessionKey[13] << 8)
                | ((uint32_t)s_sessionKey[14] << 16) | ((uint32_t)s_sessionKey[15] << 24);

    DW3000.write(SET_1_2_REG, 0x04, w0, 4);
    DW3000.write(SET_1_2_REG, 0x08, w1, 4);
    DW3000.write(SET_1_2_REG, 0x0C, w2, 4);
    DW3000.write(SET_1_2_REG, 0x10, w3, 4);

    // 2. Seed AES từ K_session vừa ghi vào register — SAU KHI register đã đúng.
    DW3000.InitCrypto();

    UWB_LOG_PRINTLN("[UWB DW3000] K_session nap vao STS_KEY thanh cong");
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
        // Priority 2 > các task core 1 khác (Logic/KeyReq/NFC/NFCProv/Unlock = 1).
        // Cùng priority 1 thì FreeRTOS time-slice mỗi tick -> đang busy-wait trong
        // cửa sổ DS-TWR mà bị TaskNFC (SPI RC522) chen vào là lỡ FINAL/FINALDATA.
        // Không chiếm CPU lâu: chờ PREPOLL vẫn vTaskDelay, các vòng busy-wait bị
        // chặn UWB_CRITICAL_WAIT_MS, giữa các chu kỳ có vTaskDelay(50).
        xTaskCreatePinnedToCore(
            RangingTask, "UWBdw3000", 4096, nullptr, 2, &s_rangingTaskHandle, 1);
    }

    UWB_LOG_PRINTLN("[UWB DW3000] Ranging started");
    return true;
}

bool UWB_StopRanging()
{
    s_ranging = false;
    s_hasDistance = false;
    UWB_LOG_PRINTLN("[UWB DW3000] Ranging stopped");
    return true;
}

bool UWB_GetLastDistance(float& outMeters)
{
    if (!s_hasDistance)
        return false;

    outMeters = s_lastDistance;
    return true;
}
