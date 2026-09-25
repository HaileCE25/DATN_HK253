#include "lcd/lcd_display.h"
#include "lcd/lcd_config.h"
#include "shared/config.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

constexpr size_t LCD_LINE_BUF = LCD_COLS + 1;

struct LcdState
{
    bool             hasCarStatus;
    CarStatusPayload car;
    uint32_t         lastCarStatusMs;

    // Mặc định LOCK: relay ở mức nghỉ sau khi gateway boot = khoá.
    bool unlocked;

    bool     msgActive;
    uint32_t msgStartMs;    // lưu start + duration (không lưu mốc kết thúc)
    uint32_t msgDurationMs; // để phép so sánh an toàn khi millis() tràn
    char     msg[LCD_ROWS][LCD_LINE_BUF];
};

static portMUX_TYPE       s_mux   = portMUX_INITIALIZER_UNLOCKED;
static LcdState           s_state = {};
static LiquidCrystal_I2C* s_lcd   = nullptr;

// Nội dung đang hiện trên LCD - chỉ ghi lại dòng nào đổi, tránh nhấp nháy
// do clear() và giảm lưu lượng I2C.
static char s_shown[LCD_ROWS][LCD_LINE_BUF] = {};

// =============================================================================
//  FORMAT
// =============================================================================
// Cắt/đệm khoảng trắng cho đủ LCD_COLS để ghi đè sạch nội dung cũ.
static void PadLine(char* line)
{
    size_t len = strnlen(line, LCD_COLS);
    memset(line + len, ' ', LCD_COLS - len);
    line[LCD_COLS] = '\0';
}

// Tối đa 7 ký tự
static const char* BleLabel(uint8_t ble)
{
    switch (ble)
    {
    case CAR_STATUS_BLE_ADVERTISING:    return "ADVERT";
    case CAR_STATUS_BLE_CONNECTED:      return "CONNECT";
    case CAR_STATUS_BLE_AUTHENTICATING: return "AUTHING";
    case CAR_STATUS_BLE_AUTHENTICATED:  return "AUTH OK";
    case CAR_STATUS_BLE_AUTH_FAIL:      return "AUTH NG";
    default:                            return "?";
    }
}

// Tối đa 5 ký tự
static const char* FsmLabel(uint8_t fsm)
{
    switch (fsm)
    {
    case CAR_STATUS_FSM_IDLE:          return "IDLE";
    case CAR_STATUS_FSM_AUTH:          return "AUTH";
    case CAR_STATUS_FSM_TRACKING:      return "TRACK";
    case CAR_STATUS_FSM_UNLOCK_WINDOW: return "HOLD";
    case CAR_STATUS_FSM_UNLOCKED:      return "UNLKD";
    case CAR_STATUS_FSM_COOLDOWN:      return "COOL";
    default:                           return "?";
    }
}

// Luôn đúng 6 ký tự: " 0.75m", "12.34m", "  >99m", "--.--m"
static void FormatDistance(char* out, size_t outSize, uint16_t distanceCm)
{
    if (distanceCm == CAR_STATUS_DISTANCE_INVALID)
        snprintf(out, outSize, "--.--m");
    else if (distanceCm >= 10000)
        snprintf(out, outSize, "  >99m");
    else
        snprintf(out, outSize, "%2u.%02um", (unsigned)(distanceCm / 100), (unsigned)(distanceCm % 100));
}

static void BuildLines(const LcdState& st, uint32_t now, char lines[LCD_ROWS][LCD_LINE_BUF])
{
    const char* lock = st.unlocked ? "OPEN" : "LOCK";

    if (st.msgActive && (now - st.msgStartMs) < st.msgDurationMs)
    {
        memcpy(lines, st.msg, sizeof(st.msg));
    }
    // Status lấy tại chỗ, chỉ thiếu trong ~100ms đầu sau boot
    else if (!st.hasCarStatus)
    {
        snprintf(lines[0], LCD_LINE_BUF, "CAR ECU     %-4s", lock);
        snprintf(lines[1], LCD_LINE_BUF, "Starting...");
    }
    else
    {
        char dist[8];
        FormatDistance(dist, sizeof(dist), st.car.distance_cm);

        snprintf(lines[0], LCD_LINE_BUF, "BLE %-7s %-4s", BleLabel(st.car.ble), lock);
        snprintf(lines[1], LCD_LINE_BUF, "UWB %s %-5s", dist, FsmLabel(st.car.fsm));
    }

    for (uint8_t r = 0; r < LCD_ROWS; r++)
        PadLine(lines[r]);
}

// =============================================================================
//  RENDER TASK - nơi DUY NHẤT truy cập I2C (kể cả Wire.begin + init HD44780)
// =============================================================================
// Init nằm trong task (không nằm trong LCD_Init) vì:
//  - ngắt I2C của ESP32 được gắn vào core gọi Wire.begin -> chạy trên
//    LCD_TASK_CORE, không chen vào core đang chạy UWB DS-TWR (car).
//  - LiquidCrystal_I2C::begin() có delay ~1s -> không block setup().
//  - cùng 1 đường code xử lý cả "chưa cắm LCD lúc boot" lẫn "rút ra cắm lại".
static uint8_t s_addr = 0;

static uint8_t FindLcdAddress()
{
    const uint8_t candidates[] = { LCD_I2C_ADDR_PRIMARY, LCD_I2C_ADDR_SECONDARY };
    for (uint8_t candidate : candidates)
    {
        Wire.beginTransmission(candidate);
        if (Wire.endTransmission() == 0)
            return candidate;
    }
    return 0;
}

static bool ProbeLcd()
{
    Wire.beginTransmission(s_addr);
    return Wire.endTransmission() == 0;
}

// Khởi tạo (lại) HD44780 và buộc vẽ lại toàn bộ. Khi LCD mất nguồn/rút dây,
// HD44780 quay về trạng thái chưa cấu hình (màn trắng hoặc hàng ô đen), còn
// s_shown vẫn tưởng nội dung cũ đang hiện -> phải xoá để vẽ lại.
static void InitLcdController()
{
    if (s_lcd == nullptr)
        s_lcd = new LiquidCrystal_I2C(s_addr, LCD_COLS, LCD_ROWS);

    s_lcd->init();
    s_lcd->backlight();
    s_lcd->clear();
    for (uint8_t r = 0; r < LCD_ROWS; r++)
        s_shown[r][0] = '\0';
}

static void TaskLcd(void* pvParameters)
{
    // PHẢI begin với chân tuỳ chỉnh TRƯỚC lcd.init(): LiquidCrystal_I2C::init()
    // tự gọi Wire.begin() không tham số. Bus đã chạy thì core ESP32 bỏ qua lần
    // gọi đó; nếu chưa, nó sẽ lấy chân mặc định của variant (GPIO8/9) - sai dây.
    if (!Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN, LCD_I2C_FREQ_HZ))
    {
        LOG_PRINTLN("[LCD ERROR] Wire.begin that bai");
        vTaskDelete(NULL);
    }

    char lines[LCD_ROWS][LCD_LINE_BUF];
    bool connected = false;
    bool loggedMissing = false;
    bool logNextRender = false; // log nội dung lần vẽ đầu để đối chiếu với màn hình thật
    uint32_t lastProbeMs = 0;
    bool firstProbe = true;

    for (;;)
    {
        uint32_t nowMs = millis();
        if (firstProbe || (nowMs - lastProbeMs) >= LCD_PROBE_MS)
        {
            firstProbe = false;
            lastProbeMs = nowMs;

            if (!connected)
            {
                // Chưa từng thấy LCD: dò cả 2 địa chỉ. Đã thấy rồi: chỉ dò địa chỉ cũ.
                uint8_t found = (s_addr == 0) ? FindLcdAddress() : (ProbeLcd() ? s_addr : 0);
                if (found != 0)
                {
                    s_addr = found;
                    InitLcdController();
                    // LOG_PRINTF("[LCD] LCD 1602 I2C OK tai 0x%02X (SDA=%d SCL=%d)\n",
                    //            s_addr, LCD_SDA_PIN, LCD_SCL_PIN);
                    connected = true;
                    loggedMissing = false;
                    logNextRender = true;
                }
                else if (!loggedMissing)
                {
                    LOG_PRINTF("[LCD ERROR] Khong tim thay LCD tai 0x%02X/0x%02X (SDA=%d SCL=%d)\n",
                               LCD_I2C_ADDR_PRIMARY, LCD_I2C_ADDR_SECONDARY, LCD_SDA_PIN, LCD_SCL_PIN);
                    loggedMissing = true;
                }
            }
            else if (!ProbeLcd())
            {
                LOG_PRINTF("[LCD ERROR] Mat ket noi I2C tai 0x%02X, cho LCD quay lai\n", s_addr);
                connected = false;
                loggedMissing = true;
            }
        }

        if (!connected)
        {
            vTaskDelay(pdMS_TO_TICKS(LCD_REFRESH_MS));
            continue;
        }

        LcdState snapshot;
        taskENTER_CRITICAL(&s_mux);
        snapshot = s_state;
        taskEXIT_CRITICAL(&s_mux);

        BuildLines(snapshot, millis(), lines);

        for (uint8_t r = 0; r < LCD_ROWS; r++)
        {
            if (strcmp(lines[r], s_shown[r]) != 0)
            {
                s_lcd->setCursor(0, r);
                s_lcd->print(lines[r]);
                memcpy(s_shown[r], lines[r], LCD_LINE_BUF);
            }
        }

        if (logNextRender)
        {
            LOG_PRINTF("[LCD] Dang hien: \"%s\" / \"%s\"\n", lines[0], lines[1]);
            logNextRender = false;
        }

        vTaskDelay(pdMS_TO_TICKS(LCD_REFRESH_MS));
    }
}

// =============================================================================
//  API
// =============================================================================
bool LCD_Init()
{
    static bool s_started = false;
    if (s_started)
        return true;

    if (xTaskCreatePinnedToCore(TaskLcd, "LCD", 4096, nullptr, LCD_TASK_PRIORITY, nullptr, LCD_TASK_CORE) != pdPASS)
    {
        LOG_PRINTLN("[LCD ERROR] Tao task LCD that bai");
        return false;
    }

    s_started = true;
    return true;
}

void LCD_SetCarStatus(const CarStatusPayload& status)
{
    uint32_t now = millis();
    taskENTER_CRITICAL(&s_mux);
    s_state.car             = status;
    s_state.hasCarStatus    = true;
    s_state.lastCarStatusMs = now;
    taskEXIT_CRITICAL(&s_mux);
}

void LCD_SetLockState(bool unlocked)
{
    taskENTER_CRITICAL(&s_mux);
    s_state.unlocked = unlocked;
    taskEXIT_CRITICAL(&s_mux);
}

void LCD_ShowMessage(const char* line0, const char* line1, uint32_t durationMs)
{
    char tmp[LCD_ROWS][LCD_LINE_BUF];
    snprintf(tmp[0], LCD_LINE_BUF, "%s", line0 ? line0 : "");
    snprintf(tmp[1], LCD_LINE_BUF, "%s", line1 ? line1 : "");
    uint32_t now = millis();

    taskENTER_CRITICAL(&s_mux);
    memcpy(s_state.msg, tmp, sizeof(tmp));
    s_state.msgStartMs    = now;
    s_state.msgDurationMs = durationMs;
    s_state.msgActive     = true;
    taskEXIT_CRITICAL(&s_mux);
}
