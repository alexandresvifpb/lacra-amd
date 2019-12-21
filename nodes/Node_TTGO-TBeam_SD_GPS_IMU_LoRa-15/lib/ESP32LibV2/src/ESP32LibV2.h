#include <Arduino.h>
#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"

#ifndef ESP32LIBV2_H 
#define ESP32LIBV2_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_PIN_VBAT                (35)        // GPIO35 - pino para leitura da Vpp da bateria

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


class ESP32Lib
{
    public:
        ESP32Lib();                // construtor

        void addNewSendMessage(String message);
        boolean avaliableSendMessage(void);
        String getNextSendMessage(void);
        String encoderJSon(String id, uint8_t type, String payload);
        String getMacAddress(void);
        float getVBat(void);

        boolean esp32DebugMain = ESP32_DEBUG_MAIN;
        boolean esp32DebugAdd = ESP32_DEBUG_ADD;
        boolean esp32DebugGet = ESP32_DEBUG_GET;

        const int UTC_offset = ESP32_UTC_OFFSET;
        uint64_t min5TaskDelayMS = ESP32_MIN5_DELAY;

    private:
        const uint8_t vbatPin = GPS_PIN_VBAT;


};

#ifdef __cplusplus
}
#endif

#endif  // ESP32LIBV2_H