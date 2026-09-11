/*
 * uwb_dw3000_tx.cpp — hiện thực include/uwb/uwb_hal.h trên DW3000, vai TX/INITIATOR.
 *
 * Board này (keyfob, env "keyfob") là con PHÁT poll. KHÔNG tính khoảng cách —
 * việc đó do phía RX (car) làm. Vì dùng chung uwb_hal.h, hàm
 * UWB_GetLastDistance() ở đây LUÔN trả false.
 * Thêm UWB_SetSessionKey() để bind K_session vào STS trước ranging.
 *
 * ── Cấu trúc file (song song với src/uwb/uwb_dw3000_rx.cpp) ───────────────────
 *  Phần A: FSM TX — port từ reference/dw3000_tx.cpp. KHÔNG đổi thuật toán.
 *  Phần B: wrapper task — cờ s_ranging bật/tắt, task nền chạy txCycleOnce().
 */

#include "uwb/uwb_hal.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "shared/config.h"
#include "DW3000_tx.h"

// timestamp_data được ĐỊNH NGHĨA trong DW3000_tx.cpp; FSM dưới đây ghi vào nó.
extern UwbTimestampData_t timestamp_data;

// =============================================================================
//  PHẦN A — FSM TX (port từ reference/dw3000_tx.cpp, KHÔNG đổi thuật toán)
// =============================================================================

#ifndef UWB_INIT_MAX_HARDRESET_RETRY
#define UWB_INIT_MAX_HARDRESET_RETRY 5
#endif

#ifndef UWB_TX_SENT_TIMEOUT_MS
#define UWB_TX_SENT_TIMEOUT_MS 2000
#endif

#ifndef UWB_DW3000_TX_DEBUG
#define UWB_DW3000_TX_DEBUG 0
#endif

#if UWB_DW3000_TX_DEBUG
#define TX_LOG_PRINTF(...)  LOG_PRINTF(__VA_ARGS__)
#define TX_LOG_PRINTLN(x)   LOG_PRINTLN(x)
#else
#define TX_LOG_PRINTF(...)  do {} while (0)
#define TX_LOG_PRINTLN(x)   do {} while (0)
#endif

// ---- txCoreInit() ----------------------------------------------------------

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
            TX_LOG_PRINTLN("[UWB TX] IDLE FAILED - retrying hard reset");
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

static bool txCoreInit()
{
    DW3000.begin();     // Init SPI
    DW3000.hardReset(); // hard reset in case that the chip wasn't disconnected from power
    delay(200);          // Wait for DW3000 chip to wake up

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        TX_LOG_PRINTLN("[UWB TX] txCoreInit: IDLE1 failed");
        return false;
    }

    DW3000.softReset(); // Reset in case that the chip wasn't disconnected from power
    delay(200);          // Wait for DW3000 chip to wake up

    if (!waitForIdleOrHardReset(UWB_INIT_MAX_HARDRESET_RETRY))
    {
        TX_LOG_PRINTLN("[UWB TX] txCoreInit: IDLE2 failed");
        return false;
    }

    DW3000.init(); // Initialize chip (write default values, calibration, etc.)
    uint32_t tx_fctrl_val = DW3000.read(GEN_CFG_AES_LOW_REG, 0x24);
    (void)tx_fctrl_val;
    tx_fctrl_val = DW3000.read(0x0e, 0x12) & 0xFFFF;
    tx_fctrl_val |= 0x100000;
    DW3000.write(0x0e, 0x12, tx_fctrl_val, 3);
    DW3000.write(0x0e, 0x16, 0x9b, 1);
    DW3000.configureAsTX();          // Configure basic settings for frame transmitting
    DW3000.setTXAntennaDelay(16385); // set default antenna delay

    DW3000.InitCrypto();
    delay(2000);
    TX_LOG_PRINTLN("[UWB TX] txCoreInit OK - initiator ready");
    return true;
}

// ---- txCycleOnce(): 1 chu kỳ ranging DS-TWR đầy đủ (vai TX) ----------------

static bool txCycleOnce()
{
    static uint8_t buffer_prepoll[127];
    static uint8_t buffer_finalData[127];

    uint32_t exact_tx_timestamp;
    long long rx_ts;
    long long tx_ts;
    uint32_t time_start;
    uint32_t time_end;
    uint32_t time_diff;
    uint32_t rx_status = 0;
    uint32_t tx_status = 0;
    uint8_t counter_resp = 0;
    uint8_t complete_sequence = 0;

    uint8_t state = IDLE;

    while (true)
    {
        DW3000.clearSystemStatus();
        switch (state)
        {
        case IDLE:
        {
            memset(buffer_prepoll, 0xAA, sizeof(buffer_prepoll));
            DW3000.putIdle();
            DW3000.SwitchPackage(SP0);
            DW3000.PreparePrepollData(buffer_prepoll);
            DW3000.Generate_KtURSK();
            DW3000.Generate_dURSK();
            DW3000.WriteRespondSTS();
            DW3000.FinalData(buffer_finalData);
            DW3000.Cleartimestamp();

            state = PREPOLL;
            break;
        }
        case PREPOLL:
        {
            complete_sequence = 0;
            TX_LOG_PRINTLN("[UWB TX] start / PREPOLL");

            DW3000.SwitchPackage(SP0);
            DW3000.setTXFrame(0, 64, buffer_prepoll);
            DW3000.setFrameLength(64);
            DW3000.standardTX();

            state = POLL;

            {
                uint32_t t0 = millis();
                while (!(tx_status = DW3000.sentFrameSucc()))
                {
                    if (millis() - t0 > UWB_TX_SENT_TIMEOUT_MS) { tx_status = 0; break; }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            if (!tx_status) { TX_LOG_PRINTLN("[UWB TX] PREPOLL send timeout"); state = IDLE; return false; }

            DW3000.DebugToggle();
            DW3000.SwitchPackage(SP3);
            DW3000.WritePollSTS_IV();
            rx_ts = DW3000.readTXTimestamp();
            exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY_4MS) >> 8;
            DW3000.writeTXDelay(exact_tx_timestamp);
            DW3000.delayedTX();
            break;
        }
        case POLL:
        {
            counter_resp = 1;
            {
                uint32_t t0 = millis();
                while (!(tx_status = DW3000.sentFrameSucc()))
                {
                    if (millis() - t0 > UWB_TX_SENT_TIMEOUT_MS) { tx_status = 0; break; }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            if (!tx_status) { TX_LOG_PRINTLN("[UWB TX] POLL send timeout"); state = IDLE; return false; }

            DW3000.DebugToggle();
            DW3000.DebugToggle();
            state = RESPOND;
            DW3000.SwitchPackage(SP3);
            tx_ts = DW3000.readTXTimestamp();
            timestamp_data.PollTimestamp_L_u32 = (uint32_t)(tx_ts & 0xFFFFFFFF);
            DW3000.WriteRespondSTS_IV(3);
            DW3000.write(0x02, 0x04, 1, 1);
            DW3000.standardRX();
            break;
        }
        case RESPOND:
        {
            time_start = millis();
            time_diff = 0;
            // GIỮ NGUYÊN từ reference: vòng chờ bị chặn <15ms, KHÔNG chèn vTaskDelay
            while ((!(rx_status = DW3000.receivedFrameSucc())) && (time_diff < 15))
            {
                time_end = millis();
                time_diff = time_end - time_start;
            }
            if (UWB_DW3000_TX_DEBUG)
            {
                uint32_t sys_lo = DW3000.read(0x00, 0x44);
                uint32_t sys_hi = DW3000.read(0x00, 0x48);
                TX_LOG_PRINTF("[UWB TX] RESPOND sys_stat=0x%X_%X\n", (unsigned)sys_hi, (unsigned)sys_lo);
            }
            if (rx_status != 0)
            {
                DW3000.SendRespond(counter_resp - 1);
            }
            DW3000.putIdle();
            DW3000.DebugToggle();
            DW3000.DebugToggle();
            DW3000.DebugToggle();
            TX_LOG_PRINTF("[UWB TX] RESPOND rx_status=%u time_diff=%u\n",
                          (unsigned)rx_status, (unsigned)time_diff);
            if (rx_status == 0)
            {
                TX_LOG_PRINTLN("[UWB TX] RESPOND timeout, restarting");
                state = IDLE;
                return false;
            }
            {
                complete_sequence = 1;
                DW3000.WriteFinalSTS_IV();
            }
            counter_resp++;
            if (complete_sequence == 1 && counter_resp < 8)
            {
                counter_resp = 8;
            }
            DW3000.WriteRespondSTS_IV(counter_resp);
            if (counter_resp < 8)
            {
                state = RESPOND;
            }
            else if ((complete_sequence == 1) && (counter_resp == 8))
            {
                DW3000.SwitchPackage(SP3);
                DW3000.WriteFinalSTS_IV();
                DW3000.write(0x02, 0x04, 1, 1);
                DW3000.standardTX();
                state = FINAL;
            }
            break;
        }
        case FINAL:
        {
            TX_LOG_PRINTLN("[UWB TX] FINAL waiting for TX done");
            time_start = millis();
            time_diff = 0;
            // GIỮ NGUYÊN: chặn <10ms, không chèn vTaskDelay.
            while (!(tx_status = DW3000.sentFrameSucc()) && (time_diff < 10))
            {
                time_end = millis();
                time_diff = time_end - time_start;
            }
            if (tx_status != 1)
            {
                if (UWB_DW3000_TX_DEBUG)
                {
                    uint32_t sys_stat = DW3000.read(0x00, 0x44);
                    TX_LOG_PRINTF("[UWB TX] FINAL TX failed! tx_status=%u sys_stat=0x%X\n",
                                  (unsigned)tx_status, (unsigned)sys_stat);
                }
                DW3000.putIdle();
                state = IDLE;
                return false;
            }
            TX_LOG_PRINTLN("[UWB TX] FINAL sent OK");
            rx_ts = DW3000.readTXTimestamp();
            DW3000.DebugToggle();
            DW3000.DebugToggle();

            timestamp_data.FinalTimestamp_L_u32 = (uint32_t)(rx_ts & 0xFFFFFFFF);
            DW3000.putIdle();
            DW3000.SwitchPackage(SP0);
            DW3000.PrepareFinalData(buffer_finalData);
            DW3000.setTXFrame(0, 107, buffer_finalData);
            DW3000.setFrameLength(107);
            DW3000.standardTX();
            state = FINALDATA;
            break;
        }
        case FINALDATA:
        {
            time_start = millis();
            time_diff = 0;
            // GIỮ NGUYÊN: chặn <10ms, không chèn vTaskDelay.
            while (!(tx_status = DW3000.sentFrameSucc()) && (time_diff < 10))
            {
                time_end = millis();
                time_diff = time_end - time_start;
            }
            if (tx_status != 1)
            {
                TX_LOG_PRINTLN("[UWB TX] FINALDATA TX failed");
                DW3000.putIdle();
                state = IDLE;
                return false;
            }

            DW3000.DebugToggle();
            DW3000.DebugToggle();
            DW3000.DebugToggle();
            DW3000.DebugToggle();
            return true;
        }
        case SENDCIR:
        {
            DW3000.SwitchPackage(SP3);
            DW3000.setTXFrame(0x00, 0x00, NULL);
            DW3000.setFrameLength(0);
            DW3000.standardTX();
            {
                uint32_t t0 = millis();
                while (!(tx_status = DW3000.sentFrameSucc()))
                {
                    if (millis() - t0 > UWB_TX_SENT_TIMEOUT_MS) { tx_status = 0; break; }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            return tx_status ? true : false;
        }
        case RECEIVECIR:
        {
            DW3000.SwitchPackage(SP0);
            DW3000.standardRX();
            {
                uint32_t t0 = millis();
                while (!(rx_status = DW3000.receivedFrameSucc_noCIA()))
                {
                    if (millis() - t0 > UWB_TX_SENT_TIMEOUT_MS) { rx_status = 0; break; }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            if (rx_status)
            {
                DW3000.ReadFristPath();
            }
            return rx_status ? true : false;
        }
        default:
        {
            DW3000.putIdle();
            DW3000.Cleartimestamp();
            DW3000.SwitchPackage(SP0);
            return false;
        }
        }
    }
}

// =============================================================================
//  PHẦN B — wrapper task, hiện thực include/uwb/uwb_hal.h
// =============================================================================

static bool s_initialized = false;
static bool s_ranging = false;
static TaskHandle_t s_txTaskHandle = nullptr;

// K_session lưu trữ, nạp vào DW3000 trước ranging
static uint8_t s_sessionKey[16] = {0};
static bool s_sessionKeySet = false;

static void TxTask(void* pvParameters)
{
    for (;;)
    {
        if (s_ranging)
        {
            bool ok = txCycleOnce();
            (void)ok;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool UWB_Init()
{
    if (s_initialized)
        return true; // idempotent

    TX_LOG_PRINTLN("[UWB TX] Init");
    if (!txCoreInit())
    {
        TX_LOG_PRINTLN("[UWB TX] Init FAILED (chip khong vao IDLE)");
        return false;
    }

    s_initialized = true;
    TX_LOG_PRINTLN("[UWB TX] Init OK");
    return true;
}

bool UWB_SetSessionKey(const uint8_t* key, size_t len)
{
    if (key == nullptr || len == 0 || len > 16)
        return false;

    memset(s_sessionKey, 0, sizeof(s_sessionKey));
    memcpy(s_sessionKey, key, len);
    s_sessionKeySet = true;

    // Tái khởi tạo STS crypto với K_session mới.
    // QUAN TRỌNG: phải ghi register TRƯỚC, rồi mới gọi InitCrypto().
    // InitCrypto() seed AES từ giá trị đang ở STS_KEY register tại thời điểm gọi.
    // Nếu gọi InitCrypto() trước → nó seed key cũ (default), ghi key mới sau vô nghĩa.

    // 1. Ghi 16 byte K_session vào STS_KEY registers TRƯỚC (SET_1_2_REG = 0x18)
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

    // 2. Seed AES từ K_session vừa ghi — SAU KHI register đã đúng.
    DW3000.InitCrypto();

    TX_LOG_PRINTLN("[UWB TX] K_session nap vao STS_KEY thanh cong");
    return true;
}

bool UWB_StartRanging()
{
    if (!s_initialized)
        return false;

    if (s_ranging)
        return true; // idempotent

    s_ranging = true;

    if (s_txTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            TxTask, "UWBtx", 4096, nullptr, 1, &s_txTaskHandle, 0);
    }

    TX_LOG_PRINTLN("[UWB TX] Ranging started");
    return true;
}

bool UWB_StopRanging()
{
    s_ranging = false;
    TX_LOG_PRINTLN("[UWB TX] Ranging stopped");
    return true;
}

bool UWB_GetLastDistance(float& outMeters)
{
    (void)outMeters;
    return false;
}
