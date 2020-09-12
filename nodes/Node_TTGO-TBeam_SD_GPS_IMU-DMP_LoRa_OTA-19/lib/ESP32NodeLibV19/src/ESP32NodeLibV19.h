#ifndef ESP32NODELIBV19_H 
#define ESP32NODELIBV19_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"
#include <WiFi.h>           // lib para configuração do Wifi
#include <ArduinoOTA.h>     // lib do ArduinoOTA 
#include <ESPmDNS.h>        // lib necessária para comunicação network
#include <WiFiUdp.h>        // lib necessária para comunicação network
#include <NTPClient.h>      // lib para sincroniza data e hora com o servidor NTP

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

// NTP
#define NTP_UTC_OFFSET              (-3)
#define NTP_TIME_ADJUSTMENT         (3600)                  // adjustment
#define NTP_UPDATE_INTERVAL         (60000)                 // updateIntervaltimeZone
#define NTP_SERVER                  ("0.br.pool.ntp.org")   // ("192.168.10.10")       //("10.0.5.70")       // "europe.pool.ntp.org"

typedef struct {
    uint32_t sdTaskDelayMS;
    uint32_t loraTaskDelayMS;
    uint32_t gpsTaskDelayMS;
    uint32_t imuTaskDelayMS;
    uint32_t timeTaskDelayMS;
} Config_t;

typedef struct {
    String id;
    String dateTime;
    uint64_t epochTimeIMU;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
    float q0;
    float q1;
    float q2;
    float q3;
    float Yam;
    float Pitch;
    float Roll;
    uint64_t epochTimeGPS;
    float latitude;
    float longitude;
    float altitude;
    float speed;
} allDatas_t;

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
        Config_t getConfig(String strConfig);
        String getAllDatas(allDatas_t datas);
        boolean requestSync(void);
        float getVBat(void);

        // const int UTC_offset = ESP32_UTC_OFFSET;
        uint64_t timeTaskDelayMS = ESP32_TIME_DELAY;
        boolean connectedOTA = false;
        String wifi_ssid = WIFI_SSID;
        String wifi_pass = WIFI_PASS;

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

#endif  // ESP32NODELIBV19_H