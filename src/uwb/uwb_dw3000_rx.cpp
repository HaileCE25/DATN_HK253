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

// Khoảng cách tối đa coi là hợp lệ (cm). Mẫu lớn hơn là timestamp hỏng, bị loại như round lỗi.
#ifndef UWB_MAX_VALID_CM
#define UWB_MAX_VALID_CM 3000.0f
#endif

// TX antenna delay của DW3000 (đơn vị 15,65 ps, thanh ghi TX_ANTD 0x01:0x04). Đây là thông số
// vật lý cần hiệu chuẩn theo từng board: giá trị CÀNG LỚN thì khoảng cách đo CÀNG NGẮN.
// Mỗi +1 đơn vị ở MỘT phía làm khoảng cách đo giảm khoảng 0,235 cm (tính theo công thức DS-TWR,
// cần xác nhận bằng đo thật). Mặc định 16385 = giá trị nhà sản xuất; đặt giá trị hiệu chuẩn qua
// build_flags của env (xem platformio.ini), không sửa ở đây.
#ifndef UWB_ANT_DELAY_TX
#define UWB_ANT_DELAY_TX 16385
#endif

// Độ dài 1 cửa sổ nghe PREPOLL trước khi coi là "không nhận được gì" (timeout).
// Mặc định 5000 ms là RẤT dài so với chu kỳ keyfob bình thường (~50-70 ms) - nếu
// thật sự không có gì tới, thà timeout sớm rồi thử lại còn hơn "đứng im" cả 5s
// mỗi lần (nhân với UWB_PREPOLL_MAX_RETRY+1 lần thì thành hàng chục giây liền,
// đúng kiểu đã thấy trong log thật khi Keyfob ngừng phát: nhiều chu kỳ 10s liên
// tiếp không có mẫu mới). Giảm giá trị này không sửa được NGUYÊN NHÂN mất tín
// hiệu, chỉ giảm thời gian "đứng im" mỗi lần thử trước khi được thử lại.
#ifndef UWB_PREPOLL_LISTEN_TIMEOUT_MS
#define UWB_PREPOLL_LISTEN_TIMEOUT_MS 5000
#endif

// Số lần PREPOLL được phép tự lặp lại (mỗi lần = 1 cửa sổ UWB_PREPOLL_LISTEN_TIMEOUT_MS)
// trước khi 1 lần đo đơn bị coi là thất bại.
#ifndef UWB_PREPOLL_MAX_RETRY
#define UWB_PREPOLL_MAX_RETRY 1
#endif

// Số lần được bỏ qua khung không phải PREPOLL (bắt phải giữa round) và nghe lại
// trước khi coi 1 lần đo là thất bại. Mỗi lần ~1 chu kỳ keyfob (~50-70 ms).
#ifndef UWB_PREPOLL_MAX_RESYNC
#define UWB_PREPOLL_MAX_RESYNC 20
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

// Biến toàn cục của driver (DW3000_rx.cpp), dùng để phát hiện kết quả cũ.
extern uint8_t counter_dis;
extern UwbMsgFinalData_t finaldata_data;
extern UwbMsgPrepollData_t prepoll_data;

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
    DW3000.setTXAntennaDelay(UWB_ANT_DELAY_TX); // antenna delay đã hiệu chuẩn (xem UWB_ANT_DELAY_TX)

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
    uint8_t prepoll_resync_count = 0;

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
                    if (millis() - t0 > UWB_PREPOLL_LISTEN_TIMEOUT_MS) { rx_status = 0; break; }
                    vTaskDelay(pdMS_TO_TICKS(1));
                };
            }
            if ((rx_status == 1) || (rx_status == 2))
            {
                DW3000.ReceivePrepoll();
                DW3000.read_payload(0x12, 0x00, 46, buffer_prepoll);

                // PrepollData() chỉ nhận khung có Data[19]==0x01 (PREPOLL thật). Car thức dậy
                // sau duty-cycle thường bật RX GIỮA một round của keyfob -> bắt phải POLL/FINAL,
                // nếu vẫn chạy tiếp thì frame counter PREPOLL là số cũ và round bị loại.
                // Gặp khung không phải PREPOLL: nghe lại để đồng bộ với round kế tiếp.
                prepoll_data.Received = 0;
                DW3000.PrepollData(buffer_prepoll);
                if (!prepoll_data.Received)
                {
                    DW3000.putIdle();
                    if (++prepoll_resync_count > UWB_PREPOLL_MAX_RESYNC)
                    {
                        s_failState    = PREPOLL;
                        s_failRxStatus = 0x100 | rx_status;
                        s_failTxStatus = prepoll_resync_count;
                        return -1.0f;
                    }
                    state = PREPOLL;
                    break;
                }

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

            // FinalData() chỉ tính lại khoảng cách khi qua kiểm tra hợp lệ (Data[19]==0x02,
            // Received==1, frame counter lệch đúng 1). Trượt thì nó im lặng bỏ qua và
            // getLastDistance() vẫn trả giá trị CŨ -> phải phát hiện, không coi là mẫu mới.
            // counter_dis tăng mỗi lần DistanceCal() chạy.
            uint8_t distCalBefore = counter_dis;
            uint32_t dbgData19    = buffer_finalData[19] | ((uint32_t)finaldata_data.Received << 8);
            DW3000.FinalData(buffer_finalData);
            uint32_t dbgFcDiff    = finaldata_data.FrameCounter_u32 - prepoll_data.FrameCounter_u32;
            bool fresh = (counter_dis != distCalBefore);
            float distance_cm = DW3000.getLastDistance();
            DW3000.Cleartimestamp();

            if (!fresh)
            {
                // "@IDLE": rx_status = Data[19] | Received<<8, tx_status = FrameCounter final - prepoll
                s_failState    = IDLE;
                s_failRxStatus = dbgData19;
                s_failTxStatus = dbgFcDiff;
                return -1.0f;
            }

            // Chặn số vô lý: timestamp hỏng vẫn có thể tính ra khoảng cách "hợp lệ" về mặt
            // toán học (vd 49-213 m). Ngoài [0, UWB_MAX_VALID_CM] coi là round lỗi.
            if (!(distance_cm >= 0.0f) || distance_cm > UWB_MAX_VALID_CM)
            {
                // "@IDLE": rx = 0x200 (đánh dấu ngoài dải), tx = khoảng cách bị loại (cm)
                s_failState    = IDLE;
                s_failRxStatus = 0x200;
                s_failTxStatus = (uint32_t)(distance_cm > 0.0f ? distance_cm : 0.0f);
                return -1.0f;
            }
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

// ---- Sliding-window median filter (N=UWB_MEASURE_SAMPLES mẫu gần nhất) ----
// Thay cho batch cũ (gom đủ N mẫu MỚI rồi lọc 1 lần -> output mỗi ~N*70ms).
// Sliding window: mỗi round cập nhật buffer tròn rồi lọc lại NGAY -> output
// mỗi round (~50-70ms, nhanh gấp ~5 lần) nhưng vẫn giữ nguyên khả năng chống
// nhiễu multipath nhờ median trên N mẫu gần nhất.

static int compareFloatAsc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static float s_sampleBufCm[UWB_MEASURE_SAMPLES];
static int   s_sampleCount    = 0;
static int   s_sampleWriteIdx = 0;

static float pushSampleAndGetMedianCm(float cm)
{
    s_sampleBufCm[s_sampleWriteIdx] = cm;
    s_sampleWriteIdx = (s_sampleWriteIdx + 1) % UWB_MEASURE_SAMPLES;
    if (s_sampleCount < UWB_MEASURE_SAMPLES) s_sampleCount++;

    float tmp[UWB_MEASURE_SAMPLES];
    memcpy(tmp, s_sampleBufCm, sizeof(float) * s_sampleCount);
    qsort(tmp, s_sampleCount, sizeof(float), compareFloatAsc);
    return tmp[s_sampleCount / 2];
}

// ---- Bộ lọc khoảng cách 3 lớp: chặn nhảy vọt -> median -> làm mượt thích ứng ----
// Mục tiêu: đứng im thì số đo đứng yên, chỉ đổi khi khoảng cách thật sự đổi.
//  1) Chặn nhảy vọt: mẫu lệch khỏi ước lượng hiện tại quá mức người đi được
//     (V_MAX * dt + MARGIN) bị coi là multipath/NLOS và BỎ QUA, không cho vào
//     median. Chỉ chấp nhận khi UWB_GATE_REACQUIRE mẫu liên tiếp cùng "nhảy" về
//     một vùng mới (nhảy thật, vd. vừa bắt lại sync sau khoảng trống).
//  2) Median UWB_MEASURE_SAMPLES mẫu gần nhất (như cũ).
//  3) Làm mượt EMA với hệ số alpha THAY ĐỔI theo độ lệch: median chỉ xê dịch
//     nhỏ (trong DEADBAND) -> alpha nhỏ, nuốt nhiễu; lệch nhiều -> alpha lớn,
//     bám nhanh khi di chuyển thật.
// Khoảng cách giữa 2 mẫu > UWB_FILTER_MAX_DT_MS (low-power đo thưa, hoặc mất
// sync lâu): mẫu cũ không còn đại diện -> khởi tạo lại bộ lọc từ mẫu mới.
#ifndef UWB_GATE_V_MAX_MPS
#define UWB_GATE_V_MAX_MPS 3.0f      // tốc độ tối đa hợp lý giữa 2 mẫu (m/s)
#endif
#ifndef UWB_GATE_MARGIN_CM
#define UWB_GATE_MARGIN_CM 15.0f     // dung sai cộng thêm cho nhiễu đo (cm)
#endif
#ifndef UWB_GATE_REACQUIRE
#define UWB_GATE_REACQUIRE 5         // số mẫu "lạ" nhất quán để chấp nhận nhảy thật
#endif
#ifndef UWB_FILTER_MAX_DT_MS
#define UWB_FILTER_MAX_DT_MS 400     // cách nhau hơn mức này -> khởi tạo lại
#endif
// Giới hạn trên cho thời gian "chưa xác nhận" (sinceGoodMs) khi tính độ rộng cửa
// chặn nhảy vọt. KHÔNG được để nới vô hạn: trong môi trường đa đường mạnh (nhiều
// round liên tiếp bị bỏ nhưng KHÔNG có khoảng trống dt lớn - full-rate vẫn chạy
// đều), sinceGoodMs cộng dồn qua hàng chục giây có thể làm cửa rộng tới mức chấp
// nhận nhầm một cụm nhiễu ổn định (đã thấy trong log thật: trôi dần từ 1.75 m lên
// kẹt cứng ở 2.58 m suốt hơn 1 phút, không qua nhánh "nhảy thật" nào cả). Chặn trần
// ở đây để cửa chỉ nới vừa đủ bắt kịp chuyển động thật trong một khoảng ngắn, không
// mở toang cho nhiễu dài hạn - nhảy thật LỚN hơn mức này vẫn phải qua UWB_GATE_REACQUIRE.
#ifndef UWB_GATE_MAX_STALE_MS
#define UWB_GATE_MAX_STALE_MS 500
#endif
#ifndef UWB_SMOOTH_DEADBAND_CM
#define UWB_SMOOTH_DEADBAND_CM 5.0f  // median lệch < mức này coi là nhiễu khi đứng im
#endif
#ifndef UWB_SMOOTH_ALPHA_STILL
#define UWB_SMOOTH_ALPHA_STILL 0.08f // alpha khi lệch nhỏ (đứng im)
#endif
#ifndef UWB_SMOOTH_ALPHA_SLOW
#define UWB_SMOOTH_ALPHA_SLOW  0.30f // alpha khi lệch vừa (đang di chuyển chậm)
#endif
#ifndef UWB_SMOOTH_ALPHA_FAST
#define UWB_SMOOTH_ALPHA_FAST  0.70f // alpha khi lệch lớn (di chuyển nhanh)
#endif

static float    s_filtCm          = 0.0f;
static bool     s_filtValid       = false;
static uint32_t s_filtLastMs      = 0;
static uint32_t s_lastGoodMs      = 0; // mốc lần cuối s_filtCm được TIN (nhận bình thường/reset),
                                        // khác s_filtLastMs (mốc lần cuối gọi hàm, kể cả mẫu bị bỏ)
static float    s_rejectAnchorCm  = 0.0f;
static uint8_t  s_rejectCount     = 0;

// Trạng thái của MẪU VỪA XỬ LÝ, để luồng [UWBRAW] cho thấy từng bước làm sạch (chỉ để log,
// không ảnh hưởng kết quả lọc):
//   s_dbgMedianCm : giá trị median sau bước 2 (khi mẫu bị bỏ/khởi tạo lại thì bằng ước lượng)
//   s_dbgAlpha    : hệ số EMA đã dùng ở bước 3 (0 = mẫu bị bỏ, 1 = khởi tạo lại)
//   s_dbgFlag     : 0 nhận bình thường | 1 bỏ mẫu (chặn nhảy vọt) | 2 khởi tạo lại
//                   (mẫu đầu / cách quá UWB_FILTER_MAX_DT_MS) | 3 chấp nhận nhảy thật
static float   s_dbgMedianCm = 0.0f;
static float   s_dbgAlpha    = 1.0f;
static uint8_t s_dbgFlag     = 2;

static void resetDistanceFilter()
{
    s_sampleCount   = 0;
    s_sampleWriteIdx = 0;
    s_filtValid     = false;
    s_rejectCount   = 0;
}

static float filterDistanceCm(float rawCm, uint32_t nowMs)
{
    uint32_t dtMs = s_filtValid ? (nowMs - s_filtLastMs) : 0;
    s_filtLastMs = nowMs;

    if (s_filtValid && dtMs > UWB_FILTER_MAX_DT_MS)
        resetDistanceFilter();

    if (!s_filtValid)
    {
        // Mẫu đầu tiên của phiên: nạp cửa sổ median và lấy chính nó làm ước lượng.
        s_filtCm     = pushSampleAndGetMedianCm(rawCm);
        s_filtValid  = true;
        s_lastGoodMs = nowMs;
        s_dbgFlag = 2; s_dbgMedianCm = s_filtCm; s_dbgAlpha = 1.0f;
        return s_filtCm;
    }

    // Lớp 1: chặn nhảy vọt so với ước lượng hiện tại. Độ rộng cửa tính theo thời gian
    // kể từ lần CUỐI ước lượng còn được tin (s_lastGoodMs), KHÔNG phải theo mẫu ngay
    // trước đó (dtMs) - nếu không, giữa một chuỗi mẫu bị bỏ liên tiếp (mỗi mẫu cách
    // nhau rất ngắn), cửa luôn hẹp y hệt dù đã "đóng băng" từ lâu, nên chuyển động
    // thật xảy ra trong lúc đó phải đợi đủ UWB_GATE_REACQUIRE mẫu KHỚP NHAU mới được
    // nhận - vừa chậm, vừa có thể lỡ nhận nhầm một cụm nhiễu trùng hợp giống nhau
    // (đã thấy trong log thật: 1 cụm ~20 round nhiễu cùng hướng làm "chấp nhận nhảy
    // thật" sai). Cửa rộng dần theo thời gian đứng yên bị nghi ngờ giúp mẫu THẬT lọt
    // qua sớm hơn qua đúng nhánh median/EMA bình thường, thay vì phải nhờ tới nhánh
    // "reacquire" vốn dễ bị lừa bởi nhiễu lặp lại nhiều round.
    uint32_t sinceGoodMs = nowMs - s_lastGoodMs;
    if (sinceGoodMs > UWB_GATE_MAX_STALE_MS)
        sinceGoodMs = UWB_GATE_MAX_STALE_MS; // trần - xem giải thích ở UWB_GATE_MAX_STALE_MS
    float maxJumpCm = UWB_GATE_V_MAX_MPS * 100.0f * ((float)sinceGoodMs / 1000.0f) + UWB_GATE_MARGIN_CM;
    if (fabsf(rawCm - s_filtCm) > maxJumpCm)
    {
        if (s_rejectCount > 0 && fabsf(rawCm - s_rejectAnchorCm) <= UWB_GATE_MARGIN_CM)
        {
            s_rejectCount++;
        }
        else
        {
            s_rejectCount    = 1;
            s_rejectAnchorCm = rawCm;
        }

        if (s_rejectCount < UWB_GATE_REACQUIRE)
        {
            s_dbgFlag = 1; s_dbgMedianCm = s_filtCm; s_dbgAlpha = 0.0f;
            return s_filtCm; // bỏ mẫu này, giữ nguyên ước lượng
        }

        // Đủ mẫu nhất quán ở vùng mới -> nhảy thật: khởi tạo lại từ mẫu này.
        resetDistanceFilter();
        s_filtCm     = pushSampleAndGetMedianCm(rawCm);
        s_filtValid  = true;
        s_lastGoodMs = nowMs;
        s_dbgFlag = 3; s_dbgMedianCm = s_filtCm; s_dbgAlpha = 1.0f;
        return s_filtCm;
    }
    s_rejectCount = 0;
    s_lastGoodMs  = nowMs;

    // Lớp 2: median cửa sổ trượt.
    float medianCm = pushSampleAndGetMedianCm(rawCm);

    // Lớp 3: EMA với alpha thích ứng theo độ lệch so với ước lượng hiện tại.
    float innovCm = medianCm - s_filtCm;
    float absInnov = fabsf(innovCm);
    float alpha = UWB_SMOOTH_ALPHA_FAST;
    if (absInnov < UWB_SMOOTH_DEADBAND_CM)
        alpha = UWB_SMOOTH_ALPHA_STILL;
    else if (absInnov < 3.0f * UWB_SMOOTH_DEADBAND_CM)
        alpha = UWB_SMOOTH_ALPHA_SLOW;

    s_filtCm += alpha * innovCm;
    s_dbgFlag = 0; s_dbgMedianCm = medianCm; s_dbgAlpha = alpha;
    return s_filtCm;
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

// ---- Zone & duty-cycle (xem thảo luận thiết kế) ----------------------------
// Vùng xa (> UWB_APPROACH_ZONE_M): chưa cần quyết định unlock -> đo thưa để
// tiết kiệm pin, log từng round lẻ (ít dữ liệu, xem riêng từng mẫu được).
// Vùng tiếp cận (<= UWB_APPROACH_ZONE_M): full-rate, log theo NHÓM
// UWB_MEASURE_SAMPLES round/lần (đỡ spam Serial, vẫn đủ để theo dõi xu hướng).
// Sau unlock (s_lowPowerMode=true): đo thưa bất kể khoảng cách - chỉ cần biết
// lúc nào vượt R_LOCK để relock, không cần độ chính xác/tốc độ cao nữa.
#ifndef UWB_APPROACH_ZONE_M
#define UWB_APPROACH_ZONE_M 2.0f
#endif

#ifndef UWB_FAR_ZONE_INTERVAL_MS
#define UWB_FAR_ZONE_INTERVAL_MS 1500
#endif

#ifndef UWB_POST_UNLOCK_INTERVAL_MS
#define UWB_POST_UNLOCK_INTERVAL_MS 2500
#endif

#ifndef UWB_FAIL_LOG_INTERVAL_MS
#define UWB_FAIL_LOG_INTERVAL_MS 5000
#endif

// 1 = log mọi round ngay khi đo xong và KHÔNG duty-cycle vùng xa (đo full-rate mọi
// khoảng cách trước unlock) để thấy khoảng cách thời gian thực. 0 = bật lại log
// theo nhóm + duty-cycle vùng xa để tiết kiệm năng lượng. Sau unlock luôn low-power.
#ifndef UWB_REALTIME_LOG
#define UWB_REALTIME_LOG 1
#endif

// Nghỉ giữa 2 round ở full-rate. Chỉ cần nhường CPU 1 tick: bước chờ PREPOLL đã tự
// vTaskDelay(1), và ngủ lâu (50 ms trước đây) làm car lỡ PREPOLL kế tiếp của keyfob.
#ifndef UWB_FULL_RATE_DELAY_MS
#define UWB_FULL_RATE_DELAY_MS 1
#endif

// 1 = (chỉ áp dụng khi UWB_REALTIME_LOG=1) in dòng distance khi giá trị lọc đổi đáng kể
// hoặc theo heartbeat; 0 = in mọi round như trước.
#ifndef UWB_LOG_ON_CHANGE
#define UWB_LOG_ON_CHANGE 1
#endif

#ifndef UWB_LOG_DEADBAND_CM
#define UWB_LOG_DEADBAND_CM 3.0f
#endif

#ifndef UWB_LOG_HEARTBEAT_MS
#define UWB_LOG_HEARTBEAT_MS 1000
#endif

// 1 = in luồng dữ liệu thô riêng cho MỌI round (dòng "[UWBRAW],ms,raw_cm,filt_cm" hoặc
// "[UWBRAW],ms,FAIL,state"), không phụ thuộc UWB_LOG_ON_CHANGE. Bật ở env:car; bộ lọc
// monitor/filter_uwb_raw.py tách khỏi màn hình và ghi ra logs/uwb_raw-*.csv.
#ifndef UWB_RAW_STREAM
#define UWB_RAW_STREAM 0
#endif

static float    s_lastLoggedCm    = 0.0f;
static uint32_t s_lastLogMs       = 0;
static bool     s_lowPowerMode    = false;
static float    s_lowPowerNearCm  = 100.0f; // đo thưa CHỈ khi distance < ngưỡng này (cm)
static uint8_t  s_logGroupCounter = 0;
static uint16_t s_failCount       = 0;
static uint8_t  s_lastFailState   = 0;
static uint32_t s_lastFailRx      = 0;
static uint32_t s_lastFailTx      = 0;
static uint32_t s_lastFailLogMs   = 0;

void UWB_SetLowPowerMode(bool enabled, float nearThresholdM)
{
    s_lowPowerMode   = enabled;
    s_lowPowerNearCm = nearThresholdM * 100.0f;
    UWB_LOG_PRINTF("[UWB DW3000] Low-power (post-unlock, < %.2f m) mode %s\n",
                   nearThresholdM, enabled ? "ON" : "OFF");
}

// Gọi lại mỗi vòng lặp (trước khi đo VÀ sau khi đo) - dùng s_lastDistance hiện có
// (của round trước, hoặc vừa cập nhật) để quyết định "có đang trong vùng gần cần
// đo thưa không". Nhờ gọi lại sau mỗi round, ngay khi khoảng cách vượt ngưỡng là
// vòng lặp SAU đó chuyển về full-rate ngay, không cần đợi lệnh nào khác.
static bool ShouldUseLowPowerNow()
{
    return s_lowPowerMode && s_hasDistance && (s_lastDistance * 100.0f < s_lowPowerNearCm);
}

// ---- Low-power (post-unlock): N round nhanh liên tiếp -> lấy MEDIAN --------------
// Một mẫu đơn lẻ mỗi UWB_POST_UNLOCK_INTERVAL_MS rất dễ bị 1 round nhiễu/đa đường
// làm sai lệch hẳn (đã thấy trong log thật: 0.45 m rồi 2.43 m chỉ sau đúng 1 chu kỳ
// 2.5s) vì ở tần suất thấp KHÔNG có sliding-window median nào bảo vệ - hai mẫu cách
// nhau hơn UWB_FILTER_MAX_DT_MS nên filterDistanceCm() luôn reset, filt = raw y hệt.
// Cách sửa: mỗi lần "thức dậy", đo nhanh UWB_POST_UNLOCK_BURST_SAMPLES round liên
// tiếp (~68 ms/round, cộng dồn không đáng kể so với chu kỳ ngủ 2.5s -> KHÔNG đổi
// ngân sách pin/tần suất, đúng như đã thống nhất tạm gác việc đổi rate), lấy MEDIAN
// các round thành công rồi mới đưa 1 giá trị vào filterDistanceCm() - loại được
// trường hợp đúng 1 round lỗi quyết định cả giá trị của chu kỳ 2.5s đó.
#ifndef UWB_POST_UNLOCK_BURST_SAMPLES
#define UWB_POST_UNLOCK_BURST_SAMPLES 3
#endif

static float measureLowPowerBurstCm()
{
    float buf[UWB_POST_UNLOCK_BURST_SAMPLES];
    uint8_t cnt = 0;

    for (uint8_t i = 0; i < UWB_POST_UNLOCK_BURST_SAMPLES; i++)
    {
        float cm = measure_once_cm();
        if (cm >= 0.0f)
        {
            buf[cnt++] = cm;
        }
        else
        {
            // Hạch toán round lỗi giống hệt nhánh lỗi ở RangingTask (measure_once_cm()
            // đã ghi s_failState/s_failRxStatus/s_failTxStatus trước khi return -1).
            s_failCount++;
#if UWB_RAW_STREAM
            UWB_LOG_PRINTF("[UWBRAW],%lu,FAIL,%s\n", (unsigned long)millis(), stateName(s_failState));
#endif
            s_lastFailState = s_failState;
            s_lastFailRx    = s_failRxStatus;
            s_lastFailTx    = s_failTxStatus;
        }
    }

    if (cnt == 0)
        return -1.0f; // cả burst đều lỗi - đã hạch toán ở trên, để nguyên như 1 round lỗi

    // Sắp xếp (tối đa vài phần tử) rồi lấy trung vị.
    for (uint8_t i = 1; i < cnt; i++)
    {
        float v = buf[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
        buf[j + 1] = v;
    }
    return buf[cnt / 2];
}

static void RangingTask(void* pvParameters)
{
    for (;;)
    {
        if (s_ranging)
        {
            bool useSlowRate = ShouldUseLowPowerNow();
            float cm = useSlowRate ? measureLowPowerBurstCm() : measure_once_cm();
            if (cm >= 0.0f)
            {
                uint32_t sampleMs = millis();
                float filtCm    = filterDistanceCm(cm, sampleMs);
                s_lastDistance  = filtCm / 100.0f;
                s_hasDistance   = true;

#if UWB_RAW_STREAM
                // Luồng dữ liệu thô RIÊNG: mỗi round thành công 1 dòng CSV có thẻ [UWBRAW]
                // (ms, raw_cm, med_cm, filt_cm, alpha, flag), độc lập với log đọc được ở dưới.
                // Cho thấy từng bước làm sạch: raw -> (chặn nhảy vọt) -> median -> EMA -> filt.
                // Bộ lọc monitor monitor/filter_uwb_raw.py tách các dòng này ra logs/uwb_raw-*.csv.
                UWB_LOG_PRINTF("[UWBRAW],%lu,%.1f,%.1f,%.1f,%.2f,%u\n",
                               (unsigned long)sampleMs, cm, s_dbgMedianCm, filtCm,
                               s_dbgAlpha, (unsigned)s_dbgFlag);
#endif

#if UWB_REALTIME_LOG
                // Log thời gian thực để debug: in giá trị ĐÃ LỌC (distance=, FSM dùng giá trị này)
                // kèm mẫu thô (raw=) để so sánh mức nhiễu. Với UWB_LOG_ON_CHANGE=1 chỉ in khi
                // giá trị lọc đổi >= UWB_LOG_DEADBAND_CM hoặc quá UWB_LOG_HEARTBEAT_MS chưa in
                // dòng nào -> đứng im thì gần như im lặng, di chuyển thì vẫn thấy đủ.
#if UWB_LOG_ON_CHANGE
                if (fabsf(filtCm - s_lastLoggedCm) >= UWB_LOG_DEADBAND_CM ||
                    (sampleMs - s_lastLogMs) >= UWB_LOG_HEARTBEAT_MS)
                {
                    s_lastLoggedCm = filtCm;
                    s_lastLogMs    = sampleMs;
                    UWB_LOG_PRINTF("[UWB DW3000] %lu ms distance=%.2f m (raw=%.2f)\n",
                                   (unsigned long)sampleMs, filtCm / 100.0f, cm / 100.0f);
                }
#else
                UWB_LOG_PRINTF("[UWB DW3000] %lu ms distance=%.2f m (raw=%.2f)\n",
                               (unsigned long)sampleMs, filtCm / 100.0f, cm / 100.0f);
#endif
#else
                if (s_lastDistance > UWB_APPROACH_ZONE_M)
                {
                    // Vùng xa: mỗi round là 1 mẫu độc lập, đủ thưa để log riêng.
                    UWB_LOG_PRINTF("[UWB DW3000] distance=%.2f m (far)\n", s_lastDistance);
                }
                else
                {
                    // Vùng tiếp cận: full-rate nên log riêng từng round sẽ spam,
                    // gộp lại in 1 dòng mỗi nhóm UWB_MEASURE_SAMPLES round.
                    s_logGroupCounter++;
                    if (s_logGroupCounter >= UWB_MEASURE_SAMPLES)
                    {
                        s_logGroupCounter = 0;
                        UWB_LOG_PRINTF("[UWB DW3000] distance=%.2f m (group median)\n", s_lastDistance);
                    }
                }
#endif
            }
            else if (!useSlowRate)
            {
                // Không in từng lần (round lỗi khá thường xuyên -> tràn log), chỉ gom đếm.
                // (Ở low-power, measureLowPowerBurstCm() đã tự hạch toán từng round lỗi
                // trong burst rồi - không lặp lại ở đây kẻo đếm trùng.)
                s_failCount++;
#if UWB_RAW_STREAM
                UWB_LOG_PRINTF("[UWBRAW],%lu,FAIL,%s\n",
                               (unsigned long)millis(), stateName(s_failState));
#endif
                s_lastFailState = s_failState;
                s_lastFailRx    = s_failRxStatus;
                s_lastFailTx    = s_failTxStatus;
            }

            uint32_t nowMs = millis();
            if (s_failCount > 0 && (nowMs - s_lastFailLogMs) >= UWB_FAIL_LOG_INTERVAL_MS)
            {
                UWB_LOG_PRINTF("[UWB DW3000] %u round loi trong %lu ms (gan nhat @%s rx=%u tx=%u)\n",
                               (unsigned)s_failCount, (unsigned long)(nowMs - s_lastFailLogMs),
                               stateName(s_lastFailState), (unsigned)s_lastFailRx, (unsigned)s_lastFailTx);
                s_failCount     = 0;
                s_lastFailLogMs = nowMs;
            }
        }

        // Gọi lại SAU khi đo (s_lastDistance có thể vừa đổi) để chọn thời gian ngủ cho
        // vòng kế tiếp - rời khỏi vùng gần là ngủ ngắn lại (full-rate) ngay từ lần sau.
        uint32_t delayMs = UWB_FULL_RATE_DELAY_MS;
        if (ShouldUseLowPowerNow())
            delayMs = UWB_POST_UNLOCK_INTERVAL_MS;
#if !UWB_REALTIME_LOG
        else if (s_hasDistance && s_lastDistance > UWB_APPROACH_ZONE_M)
            delayMs = UWB_FAR_ZONE_INTERVAL_MS;
#endif

        vTaskDelay(pdMS_TO_TICKS(delayMs));
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
        // Chẩn đoán: DEV_ID (reg 0x00:00) phải là 0xDECA0302/0xDECA0312.
        //   0xFFFFFFFF / 0x00000000 -> SPI không nói chuyện được (dây MISO/SCK/CS, nguồn 3V3/GND)
        //   0xDECA03x2 nhưng không IDLE -> SPI OK, vấn đề ở reset/clock/nguồn chip
        UWB_LOG_PRINTF("[UWB DW3000] DIAG DEV_ID=0x%08lX PMSC_STATE=0x%08lX SYS_STATUS=0x%08lX\n",
                       (unsigned long)DW3000.read(0x00, 0x00),
                       (unsigned long)DW3000.read(0x0F, 0x30),
                       (unsigned long)DW3000.read(0x00, 0x44));
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
    resetDistanceFilter();
    s_lastLoggedCm = 0.0f;
    s_lastLogMs    = 0; // dòng đầu của phiên luôn được in
    s_logGroupCounter = 0;
    s_lowPowerMode = false; // luôn bắt đầu phiên mới ở full-rate

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
