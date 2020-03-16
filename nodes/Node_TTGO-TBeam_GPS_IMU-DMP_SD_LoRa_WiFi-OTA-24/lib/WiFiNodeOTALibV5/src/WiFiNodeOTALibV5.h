#ifndef WIFINODEOTALIBV5_H
#define WIFINODEOTALIBV5_H

#include <Arduino.h>
#include <WiFi.h>           // Wifi configuration library
#include <ArduinoOTA.h>     // ArduinoOTA Library (OTA)
#include <WiFiUdp.h>        // Library required for network communication (OTA)

#ifdef __cplusplus
extern "C" {
#endif

#define STA_DELAY_MS                    (60000)

// OTA
#define WIFI_STA_SSID                   ("AMD")                 // "brisa-594111";
#define WIFI_STA_PASS                   ("amd12345678")         // "gbalklxz";
#define HOSTNAME                        ("Node-")               // ("Node-25AA24")

// WEB SERVER TERMINAL
#define WEB_SERVER_PORT                 (23)    // ServerPort

#define WIFIOTA_TASK_DELAY_MS           (1000)

class WiFiOTALib
{
    public:

        WiFiOTALib();     // constructor

        boolean begin(void);

        uint8_t listenForClientsWS(void);
        uint8_t availableClientWS(void);
        uint8_t availableRecvDataClientWS(void);
        void closeClientWS(void);
        String readClientWS(void);
        void printClientWS(String strValue);

        // // STA
        boolean initSTA(void);
        boolean isConnectedSTA(void);
        void disconnectSTA(void);

        // // OTA
        boolean activeOTA(void);

        void setMACAddress(String mac);
        uint32_t tsWiFiNodeOTATaskDelayMS = WIFIOTA_TASK_DELAY_MS;

    private:
        String wifi_ssid = WIFI_STA_SSID;
        String wifi_pass = WIFI_STA_PASS;
        boolean connectedModeSTA = false;
        String macAddress;

        boolean isActiveServiceWS = false;

};

#ifdef __cplusplus
}
#endif

#endif // WIFINODEOTALIBV5_H