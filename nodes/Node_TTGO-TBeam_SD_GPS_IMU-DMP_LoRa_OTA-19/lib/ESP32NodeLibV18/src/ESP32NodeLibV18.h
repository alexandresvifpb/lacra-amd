#include <Arduino.h>
#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"
#include <WiFi.h>           //lib para configuração do Wifi
#include <ArduinoOTA.h>     //lib do ArduinoOTA 
#include <ESPmDNS.h>        //lib necessária para comunicação network
#include <WiFiUdp.h>        //lib necessária para comunicação network

#ifndef ESP32NODELIBV18_H 
#define ESP32NODELIBV18_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_PIN_VBAT                (35)        // GPIO35 - pino para leitura da Vpp da bateria

#define ESP32_DEBUG_MAIN            (true)
#define ESP32_DEBUG_ADD             (false)
#define ESP32_DEBUG_GET             (false)

#define ESP32_UTC_OFFSET            (-3)

#define ESP32_TIME_DELAY            (120000)            // em mili segundos

#define COMMAND_EPOCH_TIME          (10)

#define COMMAND_IMU_INTERVAL        (20)
#define COMMAND_SEND_DATA_LORA      (21)

#define COMMAND_GPS_INTERVAL        (30)

#define COMMAND_LORA_INTERVAL       (40)

#define COMMAND_SDCARD_INTERVAL     (50)

// OTA
#define WIFI_SSID                   ("AMD")                 // "brisa-594111";
#define WIFI_PASS                   ("amd12345678")         // "gbalklxz";
#define HOSTNAME                    ("Node-")               // ("Node-25AA24")
#define WAITCONNECTION              (30)

class ESP32NodeLib
{
    public:
        ESP32NodeLib();                // construtor

        boolean begin(void);
        boolean connectOTA(void);
        void addNewSendMessage(String message);
        boolean avaliableSendMessage(void);
        String getNextSendMessage(void);
        String encoderJSon(String id, uint8_t type, String payload);
        String getMacAddress(void);
        float getVBat(void);

        // const int UTC_offset = ESP32_UTC_OFFSET;
        uint64_t timeTaskDelayMS = ESP32_TIME_DELAY;
        boolean connectedOTA = false;

    private:
        // boolean esp32DebugMain = ESP32_DEBUG_MAIN;
        boolean esp32DebugAdd = ESP32_DEBUG_ADD;
        boolean esp32DebugGet = ESP32_DEBUG_GET;
        const uint8_t vbatPin = GPS_PIN_VBAT;
        String hostname;

};

#ifdef __cplusplus
}
#endif

#endif  // ESP32NODELIBV18_H