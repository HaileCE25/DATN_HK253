#include <Arduino.h>
#include "uwb_hal.h"

// Mẫu test HAL: setup() gọi uwb_init(), loop() gọi uwb_get_distance() và in
// 1 dòng sạch. Chỗ này sẽ được thay bằng FSM/BLE của nhóm sau này.

void setup()
{
    if (!uwb_init())
    {
        Serial.println("[ERROR] uwb_init failed");
    }
}

void loop()
{
    float d = uwb_get_distance();
    if (d >= 0)
    {
        Serial.print("Distance: ");
        Serial.print(d, 2);
        Serial.println(" cm");
    }
    else
    {
        Serial.println("Distance: measurement failed");
    }
}
