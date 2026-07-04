#ifndef CONFIG_H
#define CONFIG_H

/*=====================================================
                    BLE UUID
=====================================================*/

#define DEVICE_NAME_CAR     "ESP32_CAR_ECU"
#define DEVICE_NAME_KEYFOB  "ESP32_KEYFOB"

#define SERVICE_UUID \
"4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define CHARACTERISTIC_UUID_RX \
"beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define CHARACTERISTIC_UUID_TX \
"1c4224ce-8d26-444a-9366-a4968848db7f"

/*=====================================================
                Protocol Configuration
=====================================================*/

#define MAX_PACKET_SIZE     128
#define BLE_QUEUE_LENGTH    10
#define BLE_TIMEOUT_MS      3000
#define BLE_CONNECT_RSSI_THRESHOLD (-75)
#endif