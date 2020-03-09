#ifndef WIFIGATEWAYLIBV1_H
#define WIFIGATEWAYLIBV1_H

#include <Arduino.h>
#include <WiFi.h>           // lib para configuração do Wifi
#include <ArduinoOTA.h>     // lib do ArduinoOTA (OTA)
#include <WiFiUdp.h>        // lib necessária para comunicação network (OTA)
#include <NTPClient.h>      // lib para sincroniza data e hora com o servidor NTP
#include <PubSubClient.h>   // MQTT
#include <ArduinoJson.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFIOTA_TASK_DELAY_MS           (1000)

#define STA_DELAY_MS                    (60000)
#define AP_DELAY_MS                     (60000)

#define INIT_UNIXTIME                   (1578410676)
#define END_UNIXTIME                    (1641164807)

// OTA
#define WIFI_STA_SSID                   ("AMD")                 // "brisa-594111";
#define WIFI_STA_PASS                   ("amd12345678")         // "gbalklxz";
#define WIFI_AP_SSID                    ("Node-")               // "brisa-594111";
#define WIFI_AP_PASS                    ("12345678")            // "gbalklxz";
#define HOSTNAME                        ("Node-")               // ("Node-25AA24")

// NTP
#define NTP_UTC_OFFSET                  (-3)
#define NTP_TIME_ADJUSTMENT             (3600)                  // adjustment
#define NTP_UPDATE_INTERVAL             (60000)                 // updateIntervaltimeZone
#define NTP_SERVER                      ("0.br.pool.ntp.org")   // ("192.168.10.10")       //("10.0.5.70")       // "europe.pool.ntp.org"
#define NTP_LOOP_UPDATE                 (5)



// MQTT
#define MQTT_TOPIC                      ("AMD")
#define MQTT_SERVER                     ("192.168.10.10")      //("192.168.10.50") ("10.0.5.50")       // ("192.168.1.2") // ("ec2-52-14-53-218.us-east-2.compute.amazonaws.com")   //  ("192.168.1.7")   //  ("150.165.82.50")
#define MQTT_PORT                       (1883)              //(1883)              //  (995)
#define MQTT_MAX_TRANSFER_SIZE          (180)
#define MQTT_DEBUG_MAIN                 (false)
#define MQTT_DEBUG_BEGIN                (false)
#define MQTT_DEBUG_LOOP                 (false)
#define MQTT_DEBUG_TIME_SERIAL          (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define MQTT_TASK_DELAY_MS              (200)
#define MQTT_RECV_TASK_DELAY_MS         (200)
#define MQTT_DEBUG_MESSAGE              ("MQTT_TASK: ")

// WEB SERVER TERMINAL
#define WEB_SERVER_PORT                 (23)    // ServerPort

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageMQTT_t;

class WiFiGatewayLib
{
    public:

        WiFiGatewayLib();     // construtor

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

class MQTTLib
{
    public:
        MQTTLib();            // construtor

        // MQTT
        boolean bootMQTT(void);
        boolean initMQTT(void);
        void loopMQTT(void);
        boolean reconnectMQTT(void);
        void sendMQTT(String value);
        String getReceivedMQTT(void);
        boolean isNewRecvMsgMQTT(void);
        boolean isConnectedMQTT(void);
        boolean checkWiFiConnectionMQTT(void);
        // boolean connectWiFi(void);
        boolean checkInternetMQTT(void);
        uint64_t getEpochTimeMQTT(void);
        // String getMacAddress(void);
        messageMQTT_t decoderStrJSonInObjectMsgMQTT(String strJson);

        boolean mqttDebugMain;
        boolean mqttDebugBegin;
        boolean mqttDebugLoop;
        boolean wifiDebug;
        uint16_t mqttDebugTimeSerial;
        uint64_t mqttTaskDelayMS;
        uint64_t recvMqttTaskDelayMS;

    private:

        // MQTT
        // const char* ssid     = WIFI_SSID;
        // const char* password = WIFI_PASS;
        const char* mqtt_server = MQTT_SERVER;
        const int mqtt_port = MQTT_PORT;
        // String macAddress;
                
};

#ifdef __cplusplus
}
#endif

#endif // WIFIGATEWAYLIBV1_H