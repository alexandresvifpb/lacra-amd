#include <WiFi.h>           //lib para configuração do Wifi
#include <PubSubClient.h>   // MQTT

#ifndef MQTTLibV2_h 
#define MQTTLibV2_h

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_TOPIC              ("AMD")
#define MQTT_SERVER             ("192.168.10.50")       //("192.168.10.50")        // ("192.168.1.2") // ("ec2-52-14-53-218.us-east-2.compute.amazonaws.com")   //  ("192.168.1.7")   //  ("150.165.82.50")
#define MQTT_PORT               (1883)                   //(1883)              //  (995)
#define WIFI_DEBUG              (true)
#define MQTT_MAX_TRANSFER_SIZE  180

#define WIFI_SSID               ("AMD")                 // "brisa-594111";
#define WIFI_PASS               ("amd12345678")         // "gbalklxz";

class MQTTLib
{
    public:
        MQTTLib();            // construtor

        bool begin(void);
        bool init(void);
        void loop(void);
        bool reconnect(void);
        void send(String value);
        String getReceivedMQTT(void);
        bool isConnected(void);
        boolean checkWiFiConnection(void);
        boolean connectWiFi(void);
        boolean checkInternet(void);

    private:
        // String oledMqttServerStatus;
        const char* ssid     = WIFI_SSID;
        const char* password = WIFI_PASS;
        const char* mqtt_server = MQTT_SERVER;
        const int mqtt_port = MQTT_PORT;
        
};

#ifdef __cplusplus
}
#endif

#endif  // MQTTLibV2_h