// =============================================================================
//  LCD BRING-UP TEST - Hello World (env:lcd_test)
// =============================================================================
// Firmware tối giản chỉ để kiểm tra LCD 1602 I2C + dây nối, KHÔNG có BLE/UWB/CAN.
// Nạp vào board car (COM8): pio run -e lcd_test -t upload
// Xong thì nạp lại firmware thật: pio run -e car -t upload
//
// Chân giống env:car: SDA=GPIO11, SCL=GPIO12.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

constexpr int LCD_SDA_PIN = 11;
constexpr int LCD_SCL_PIN = 12;

static LiquidCrystal_I2C* s_lcd = nullptr;

// Quét toàn bộ bus - in mọi địa chỉ ACK, không chỉ 0x27/0x3F, để thấy ngay
// LCD có địa chỉ lạ hay bus không có thiết bị nào (dây/nguồn).
static uint8_t ScanI2C()
{
    uint8_t firstFound = 0;
    uint8_t count = 0;

    Serial.println("[LCD TEST] Quet I2C...");
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf("[LCD TEST]   Thiet bi tai 0x%02X\n", addr);
            if (firstFound == 0)
                firstFound = addr;
            count++;
        }
    }
    Serial.printf("[LCD TEST] Tim thay %u thiet bi\n", count);
    return firstFound;
}

void setup()
{
    Serial.begin(115200);
    delay(1500); // chờ USB CDC để kịp thấy log đầu

    Serial.println();
    Serial.println("===============================");
    Serial.println("   LCD TEST - HELLO WORLD");
    Serial.println("===============================");
    Serial.printf("[LCD TEST] SDA=GPIO%d SCL=GPIO%d\n", LCD_SDA_PIN, LCD_SCL_PIN);

    // Begin với chân tuỳ chỉnh TRƯỚC lcd.init() (init() tự gọi Wire.begin() mặc định)
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN, 100000);

    uint8_t addr = ScanI2C();
    if (addr == 0)
    {
        Serial.println("[LCD TEST] KHONG co thiet bi I2C -> kiem tra GND chung, VCC 5V, SDA/SCL co bi dao khong");
        return;
    }

    s_lcd = new LiquidCrystal_I2C(addr, 16, 2);
    s_lcd->init();
    s_lcd->backlight();
    s_lcd->clear();

    s_lcd->setCursor(0, 0);
    s_lcd->print("Hello World");
    s_lcd->setCursor(0, 1);
    s_lcd->print("LCD 1602 OK");

    Serial.printf("[LCD TEST] Da ghi \"Hello World\" ra LCD tai 0x%02X\n", addr);
    Serial.println("[LCD TEST] Neu man hinh trang: xoay bien tro contrast phia sau");
}

void loop()
{
    if (s_lcd == nullptr)
    {
        delay(1000);
        return;
    }

    // Bộ đếm giây ở cuối dòng 2: số nhảy = LCD đang nhận lệnh liên tục
    static uint32_t lastMs = 0;
    if (millis() - lastMs >= 1000)
    {
        lastMs = millis();
        s_lcd->setCursor(12, 1);
        s_lcd->printf("%4lu", (unsigned long)(millis() / 1000) % 10000);
    }
}
