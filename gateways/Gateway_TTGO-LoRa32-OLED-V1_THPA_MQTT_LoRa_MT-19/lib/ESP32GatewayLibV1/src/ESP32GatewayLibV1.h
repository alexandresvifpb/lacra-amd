#include <Arduino.h>
#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"

#ifndef ESP32GATEWAYLIBV1_H 
#define ESP32GATEWAYLIBV1_H

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_DEBUG_MAIN            (true)
#define ESP32_DEBUG_ADD             (false)
#define ESP32_DEBUG_GET             (false)

#define ESP32_UTC_OFFSET            (-3)

#define ESP32_MIN5_DELAY            (300000)            // em mili segundos

#define COMMAND_EPOCH_TIME          (10)

#define COMMAND_IMU_INTERVAL        (20)
#define COMMAND_SEND_DATA_LORA      (21)

#define COMMAND_GPS_INTERVAL        (30)

#define COMMAND_LORA_INTERVAL       (40)

#define COMMAND_SDCARD_INTERVAL     (50)

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageRecv_t;

class ESP32GatewayLib
{
    public:
        ESP32GatewayLib();                // construtor

        String encoderMsgInStrJSon(messageRecv_t message);
        messageRecv_t decoderStrJSonInObjectMsg(String strJson);
        // messageRecv_t createObjectMsg(String id, uint8_t type, String payload);
        // String getMacAddress(void);

        // boolean esp32DebugMain = ESP32_DEBUG_MAIN;
        // boolean esp32DebugAdd = ESP32_DEBUG_ADD;
        // boolean esp32DebugGet = ESP32_DEBUG_GET;

        // const int UTC_offset = ESP32_UTC_OFFSET;
        // uint64_t min5TaskDelayMS = ESP32_MIN5_DELAY;

    private:


};

#ifdef __cplusplus
}
#endif

#endif  // ESP32GATEWAYLIBV1_H