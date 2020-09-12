#ifndef WIFIOTALIBV4_H
#define WIFIOTALIBV4_H

#include <Arduino.h>
#include <WiFi.h>           // lib para configuração do Wifi
#include <ArduinoOTA.h>     // lib do ArduinoOTA (OTA)
#include <WiFiUdp.h>        // lib necessária para comunicação network (OTA)
#include <NTPClient.h>      // lib para sincroniza data e hora com o servidor NTP

#ifdef __cplusplus
extern "C" {
#endif

#define WIFIOTA_TASK_DELAY_MS           (1000)

const uint16_t tsWiFiOTATaskDelayMS = WIFIOTA_TASK_DELAY_MS;

#define STA_DELAY_MS                    (60000)
#define AP_DELAY_MS                     (60000)

// #define INIT_UNIXTIME                   (1578410676)
// #define END_UNIXTIME                    (1641164807)

// OTA
<<<<<<< HEAD
#define WIFI_STA_SSID                   ("brisa-594111")                 // "brisa-594111" "AMD";
#define WIFI_STA_PASS                   ("gbalklxz")         // "gbalklxz" "amd12345678";
=======
#define WIFI_STA_SSID                   ("AMD")                 // "brisa-594111";
#define WIFI_STA_PASS                   ("amd12345678")         // "gbalklxz";
>>>>>>> ebe428416b04ded786b191fcd4a8a20633cc0f5b
#define WIFI_AP_SSID                    ("Node-")               // "brisa-594111";
#define WIFI_AP_PASS                    ("12345678")            // "gbalklxz";
#define HOSTNAME                        ("Node-")               // ("Node-25AA24")

// NTP
#define NTP_UTC_OFFSET                  (-3)
#define NTP_TIME_ADJUSTMENT             (3600)                  // adjustment
#define NTP_UPDATE_INTERVAL             (60000)                 // updateIntervaltimeZone
#define NTP_SERVER                      ("0.br.pool.ntp.org")   // ("192.168.10.10")       //("10.0.5.70")       // "europe.pool.ntp.org"

// WEB SERVER TERMINAL
#define WEB_SERVER_PORT                 (23)    // ServerPort

class WiFiOTALib
{
    public:

        WiFiOTALib();     // construtor

        boolean begin(void);
        uint8_t listenForClientsWS(void);
        uint8_t availableClientWS(void);
        uint8_t availableRecvDataClientWS(void);
        void closeClientWS(void);
        String readClientWS(void);
        void printClientWS(String strValue);
        String getMacAddress(void);

        // STA
        boolean initSTA(void);
        boolean isConnectedSTA(void);
        void disconnectSTA(void);

        // OTA
        boolean activeOTA(void);

        // NTP
        boolean bootServiceNTP(void);
        long getUnixTimeNTP(void);

        // AP
        boolean initAP(void);
        boolean isConnectedAP(void);
        void disconnectAP(void);

    private:
        String wifi_ssid = WIFI_STA_SSID;
        String wifi_pass = WIFI_STA_PASS;
        boolean connectedModeSTA = false;

        String wifi_apssid = WIFI_AP_SSID;
        String wifi_appass = WIFI_AP_PASS;
        boolean connectedModeAP = false;

        boolean isActiveServiceWS = false;
        boolean isConnectedClientAP = false;
        
        // NTP
        // boolean connectedNTP = false;
        uint32_t timeOffsetNTP = NTP_UTC_OFFSET * NTP_TIME_ADJUSTMENT;
        uint32_t updateIntervalNTP = NTP_UPDATE_INTERVAL;

};

#ifdef __cplusplus
}
#endif

#endif // WIFIOTALIBV4_H