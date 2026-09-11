#include "cloud/wifi_manager.h"
#include "cloud/cloud_config.h"
#include <WiFi.h>
#include <Arduino.h>
#include "shared/config.h"

bool WiFi_Connect()
{
    LOG_PRINTF("[WIFI] Connecting to %s", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t startTime = millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime > WIFI_CONNECT_TIMEOUT_MS)
        {
            LOG_PRINTLN("[WIFI ERROR] Connect timeout");
            return false;
        }
        Serial.print(".");
        delay(300);
    }

    Serial.println();
    LOG_PRINTF("[WIFI] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

bool WiFi_IsConnected()
{
    return WiFi.status() == WL_CONNECTED;
}