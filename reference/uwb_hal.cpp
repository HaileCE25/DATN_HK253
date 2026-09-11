/*
 * uwb_hal.cpp — implementation. Xem uwb_hal.h cho mô tả interface.
 *
 * Nguồn: chuỗi state PREPOLL->POLL->RESPOND->FINAL->FINALDATA->IDLE được port
 * NGUYÊN VẸN từ main.cpp gốc (switch-case trong loop()). Chỉ 2 điểm thay đổi
 * luồng điều khiển:
 *   - case IDLE thành công  -> return distance ngay (thay vì quay lại PREPOLL
 *     để chạy tiếp vô hạn như bản gốc).
 *   - case default (state=100, timeout ở POLL/RESPOND/FINAL/FINALDATA) -> return
 *     -1 ngay (thay vì quay lại PREPOLL để tự retry vô hạn).
 * PREPOLL timeout (chờ TX) vẫn tự lặp lại đúng như gốc, nhưng bị chặn số lần
 * lặp bởi UWB_PREPOLL_MAX_RETRY (đã thống nhất với người dùng) để hàm không
 * treo cứng vĩnh viễn khi không có TX.
 */
#include "uwb_hal.h"
#include "DW3000.h"

// ---- uwb_init() ------------------------------------------------------------

// Chờ chip vào IDLE; nếu quá 50 lần check (100ms/lần, ~5s) thì hardReset và
// thử lại, tối đa maxHardResets lần. Y hệt 2 khối while giống nhau trong
// setup() gốc, gộp lại để tránh lặp code và thêm giới hạn tổng.
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

bool uwb_init()
{
    Serial.begin(115200); // Init Serial
    DW3000.begin();       // Init SPI
    DW3000.hardReset();   // hard reset in case that the chip wasn't disconnected from power
    delay(200);            // Wait for DW3000 chip to wake up
    if (UWB_DEBUG) Serial.println("debug1");

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        if (UWB_DEBUG) Serial.println("[ERROR] uwb_init: IDLE1 failed after max hard-reset retries");
        return false;
    }
    if (UWB_DEBUG) Serial.println("debug2 - IDLE1 OK");

    DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
    delay(200);          // Wait for DW3000 chip to wake up

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        if (UWB_DEBUG) Serial.println("[ERROR] uwb_init: IDLE2 failed after max hard-reset retries");
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

// Chạy đúng chuỗi state PREPOLL->POLL->RESPOND->FINAL->FINALDATA->IDLE của
// main.cpp gốc. Trả về khoảng cách (cm) khi tới IDLE thành công, hoặc -1 khi
// thất bại (timeout ở POLL/RESPOND/FINAL/FINALDATA, hoặc PREPOLL không nhận
// được TX sau UWB_PREPOLL_MAX_RETRY lần chờ).
static float measure_once_cm()
{
    uint8_t state = PREPOLL;
    uint8_t prepoll_retry_count = 0;

    uint32_t exact_tx_timestamp;
    long long rx_ts;
    uint32_t time_start = 0;
    uint32_t time_end;
    uint32_t time_diff;
    uint32_t rx_status;
    uint32_t tx_status;
    uint8_t buffer_prepoll[127];
    uint8_t buffer_finalData[127];

    while (true)
    {
        DW3000.clearSystemStatus();
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
                };
            }
            if ((rx_status == 1) || (rx_status == 2))
            {
                DW3000.ReceivePrepoll();
                DW3000.read_payload(0x12, 0x00, 46, buffer_prepoll); // Read RX_FRAME buffer0
                // Parse prepoll and compute crypto BEFORE using STS IVs
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
                // Timeout - retry PREPOLL (bounded: đây là điểm khác duy nhất so
                // với bản gốc, vốn retry vô hạn vì loop() Arduino không có khái
                // niệm "return")
                if (UWB_DEBUG) Serial.println("[WARN] PREPOLL timeout, retrying...");
                DW3000.putIdle();
                prepoll_retry_count++;
                if (prepoll_retry_count > UWB_PREPOLL_MAX_RETRY)
                {
                    return -1.0f; // không có TX sau nhiều lần chờ - bỏ cuộc chu kỳ này
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
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.receivedFrameSucc_test()))
                {
                    if (millis() - t0 > 2000) { rx_status = 0; break; } // 2s timeout
                }; // Wait until frame was received
            }
            if ((rx_status == 1))
            {
                rx_ts = DW3000.readRXTimestamp_Poll();

                DW3000.Receivepoll();

                time_end = millis();
                time_diff = time_end - time_start;
                exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY_12MS) >> 8;
                DW3000.WriteRespondSTS_IV();
                DW3000.write(0x02, 0x04, 1, 1); // Reload STS AFTER writing respond IV
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
                uint32_t t0 = millis();
                while (!(tx_status = DW3000.sentFrameSucc()))
                {
                    if (millis() - t0 > 2000) { tx_status = 0; break; }
                };
            }
            if (!tx_status) { state = 100; break; }
            DW3000.SendRespond();
            DW3000.putIdle();
            {
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.checkForIDLE()))
                {
                    if (millis() - t0 > 2000) { rx_status = 0; break; }
                }
            }
            if (!rx_status) { state = 100; break; }
            DW3000.EnableTimeout();
            DW3000.WriteTimeOutPeriod(10000);
            rx_ts = DW3000.readTXTimestamp();
            DW3000.SwitchPackage(SP3);
            DW3000.WriteFinalSTS_IV();
            DW3000.write(0x02, 0x04, 1, 1);
            // Use standardRX instead of delayedRX — TX FINAL timing is variable
            DW3000.standardRX();
            time_start = millis();
            state = FINAL;
            break;
        }
        case FINAL:
        {
            if (UWB_DEBUG) Serial.println("FINAL");

            {
                uint32_t t0 = millis();
                while ((!(rx_status = DW3000.receivedFrameSucc_test())))
                {
                    if (millis() - t0 > 2000) { rx_status = 0; break; }
                }; // Wait until frame was received
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
                    if (millis() - t0 > 2000) { rx_status = 0; break; }
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
            // Crypto now computed in PREPOLL after receiving prepoll data
            DW3000.setTXFrame(0x00);
            DW3000.setFrameLength(0);
            if (UWB_DEBUG) { Serial.print("IDLE: finalData[19]=0x"); Serial.println(buffer_finalData[19], HEX); }
            DW3000.FinalData(buffer_finalData); // parses buffer + tính DS-TWR (DistanceCal) nếu hợp lệ
            float distance_cm = DW3000.getLastDistance();
            DW3000.Cleartimestamp();

            // Thành công: trả kết quả ngay, khác bản gốc (quay lại PREPOLL chạy
            // vô hạn trong loop()). Vòng đo tiếp theo do uwb_get_distance() gọi lại.
            return distance_cm;
        }
        default: // state == 100: timeout thật ở POLL/RESPOND/FINAL/FINALDATA
        {
            DW3000.putIdle();
            DW3000.Cleartimestamp();
            DW3000.SwitchPackage(SP0);
            DW3000.setTXFrame(0x00);
            DW3000.setFrameLength(0);

            // Thất bại: trả -1 ngay, khác bản gốc (quay lại PREPOLL tự retry vô
            // hạn). Việc có thử lại hay không do uwb_get_distance() quyết định.
            return -1.0f;
        }
        }
    }
}

// ---- uwb_get_distance(): lọc median trên N chu kỳ đo ------------------------

static int compareFloatAsc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

float uwb_get_distance()
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
    }

    // Quá nửa số mẫu fail -> không đủ tin cậy để lấy median
    if (valid * 2 <= UWB_MEASURE_SAMPLES)
    {
        return -1.0f;
    }

    qsort(samples, valid, sizeof(float), compareFloatAsc);
    return samples[valid / 2]; // median của các mẫu hợp lệ
}
