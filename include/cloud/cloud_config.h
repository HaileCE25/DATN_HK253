#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
// Cloud Subsystem Configuration (Gateway only)
// -----------------------------------------------------------------------------

#define WIFI_SSID       "DUY_DAT"
#define WIFI_PASSWORD   "20001967"

#define FIREBASE_DATABASE_URL  "https://smart-car-rental-b2b-default-rtdb.asia-southeast1.firebasedatabase.app"

// Web API Key - Dùng cho Anonymous Authentication.
#define FIREBASE_API_KEY  "AIzaSyDxuUuzu-L5cippJqnqujOCLztAj-1smEk"

constexpr uint8_t FIREBASE_MASTER_KEY[32] = {
    'M', 'y', 'S', 'u', 'p', 'e', 'r', 'S',
    'e', 'c', 'r', 'e', 't', 'M', 'a', 's',
    't', 'e', 'r', 'K', 'e', 'y', '3', '2',
    'B', 'y', 't', 'e', 's', '!', '!', '!'
};

// Timeout chờ WiFi connect trước khi coi là fail.
#define WIFI_CONNECT_TIMEOUT_MS  15000