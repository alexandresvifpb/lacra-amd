#include <WiFi.h>           //lib para configuração do Wifi
#include <PubSubClient.h>   // MQTT
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

#ifndef MQTTLibV5_H 
#define MQTTLibV5_H

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_TOPIC              ("AMD")
#define MQTT_SERVER             ("192.168.1.12")      //("192.168.10.50") ("10.0.5.50")       // ("192.168.1.2") // ("ec2-52-14-53-218.us-east-2.compute.amazonaws.com")   //  ("192.168.1.7")   //  ("150.165.82.50")
#define MQTT_PORT               (1883)              //(1883)              //  (995)
#define MQTT_MAX_TRANSFER_SIZE  (180)
#define MQTT_DEBUG_MAIN         (false)
#define MQTT_DEBUG_BEGIN        (false)
#define MQTT_DEBUG_LOOP         (false)
#define MQTT_DEBUG_TIME_SERIAL  (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define MQTT_TASK_DELAY_MS      (200)
#define MQTT_RECV_TASK_DELAY_MS (200)
#define MQTT_DEBUG_MESSAGE      ("MQTT_TASK: ")

#define WIFI_SSID               ("brisa-594111")             // ("brisa-594111") ("RPi3_AMD") ("AMD")
#define WIFI_PASS               ("gbalklxz")     // ("gbalklxz") ("amd12345678") ("amd123456")
#define WIFI_DEBUG              (true)

#define NTP_UTC_OFFSET          (-3)
#define NTP_TIME_ADJUSTMENT     (3600)              // adjustment
#define NTP_UPDATE_INTERVAL     (60000)             // updateIntervaltimeZone
#define NTP_SERVER              ("192.168.10.10")       //("10.0.5.70")       // "europe.pool.ntp.org"
#define NTP_LOOP_UPDATE         (5)

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageMQTT_t;

class MQTTLib
{
    public:
        MQTTLib();            // construtor

        boolean begin(void);
        boolean init(void);
        void loop(void);
        boolean reconnect(void);
        void send(String value);
        String getReceivedMQTT(void);
        boolean isNewRecvMsgMQTT(void);
        boolean isConnected(void);
        boolean checkWiFiConnection(void);
        boolean connectWiFi(void);
        boolean checkInternet(void);
        uint64_t getEpochTime(void);
        String getMacAddress(void);
        messageMQTT_t decoderStrJSonInObjectMsgMQTT(String strJson);

        boolean mqttDebugMain;
        boolean mqttDebugBegin;
        boolean mqttDebugLoop;
        boolean wifiDebug;
        uint16_t mqttDebugTimeSerial;
        uint64_t mqttTaskDelayMS;
        uint64_t recvMqttTaskDelayMS;

    private:
        // String oledMqttServerStatus;
        const char* ssid     = WIFI_SSID;
        const char* password = WIFI_PASS;
        const char* mqtt_server = MQTT_SERVER;
        const int mqtt_port = MQTT_PORT;
        String macAddress;
                
};

#ifdef __cplusplus
}
#endif

#endif  // MQTTLibV5_H