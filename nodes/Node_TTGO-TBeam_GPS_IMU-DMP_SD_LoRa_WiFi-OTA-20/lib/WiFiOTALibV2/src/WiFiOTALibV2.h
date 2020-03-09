#ifndef WIFIOTALIBV2_H
#define WIFIOTALIBV2_H

#include <Arduino.h>
#include <WiFi.h>           // lib para configuração do Wifi
#include <ArduinoOTA.h>     // lib do ArduinoOTA (OTA)
// #include <ESPmDNS.h>        // lib necessária para comunicação network (OTA)
#include <WiFiUdp.h>        // lib necessária para comunicação network (OTA)
#include <NTPClient.h>      // lib para sincroniza data e hora com o servidor NTP

#ifdef __cplusplus
extern "C" {
#endif

#define BLEWIFIOTA_TASK_DELAY_MS        (1000)

#define TIME_OFF_SET_NTP                ()

// OTA
#define WIFI_SSID                       ("AMD")                 // "brisa-594111";
#define WIFI_PASS                       ("amd12345678")         // "gbalklxz";
#define HOSTNAME                        ("Node-")               // ("Node-25AA24")

// NTP
#define NTP_UTC_OFFSET                  (-3)
#define NTP_TIME_ADJUSTMENT             (3600)                  // adjustment
#define NTP_UPDATE_INTERVAL             (60000)                 // updateIntervaltimeZone
#define NTP_SERVER                      ("0.br.pool.ntp.org")   // ("192.168.10.10")       //("10.0.5.70")       // "europe.pool.ntp.org"

// WEB SERVER TERMINAL
#define WEB_SERVER_PORT                 (23)    // ServerPort

// BLE Bluetooth

class WiFiOTALib
{
    public:

        WiFiOTALib();     // construtor

        boolean begin(void);
        boolean reconnectWiFi(void);
        boolean isConnectedWiFi(void);

        void setSSID(String ssid);
        void setPASS(String pass);
        String getMacAddress(void);

        boolean activeOTA(void);

        // NTP
        boolean bootServiceNTP(void);
        long getUnixTimeNTP(void);

        // WEB SERVER
        uint8_t listenForClientsSocket(void);
        boolean availableConnection(void);
        int availableRecvData(void);
        void printWebSocket(String strValue);
        String readWebSocket(void);
        void closeWebSocket(void);

        // BLE

    private:
        String wifi_ssid = WIFI_SSID;
        String wifi_pass = WIFI_PASS;
        boolean wifiConnected = false;

        // OTA
        // void startOTA(void);
        // void endOTA(void);
        // void progressOTA(unsigned int progress, unsigned int total);
        // void errorOTA(ota_error_t error);

        String hostname;

        // NTP
        boolean ntpConnected = false;
        uint32_t timeOffsetNTP = NTP_UTC_OFFSET * NTP_TIME_ADJUSTMENT;
        uint32_t updateIntervalNTP = NTP_UPDATE_INTERVAL;

        // WEB SERVER
        // const uint ServerPort = WEB_SERVER_PORT;

        // BLE

};

#ifdef __cplusplus
}
#endif

#endif // WIFIOTALIBV2_H