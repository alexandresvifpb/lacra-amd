#include <Arduino.h>
#include "ArduinoJson.h"
#include "LinkedList.h"

#ifndef ESP32LIBV2_H 
#define ESP32LIBV2_H

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_DEBUG_MAIN            (true)
#define ESP32_DEBUG_ADD             (false)
#define ESP32_DEBUG_GET             (false)

class ESP32Lib
{
    public:
        ESP32Lib();                // construtor

        void addNewSendMessage(String message);
        boolean avaliableSendMessage(void);
        String getNextSendMessage(void);
        String encoderJSon(String id, uint8_t type, String payload);
        String getMacAddress(void);

        boolean esp32DebugMain = ESP32_DEBUG_MAIN;
        boolean esp32DebugAdd = ESP32_DEBUG_ADD;
        boolean esp32DebugGet = ESP32_DEBUG_GET;

    private:


};

#ifdef __cplusplus
}
#endif

#endif  // ESP32LIBV2_H