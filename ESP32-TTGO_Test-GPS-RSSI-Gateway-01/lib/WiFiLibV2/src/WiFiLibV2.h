#include <WiFi.h>
#include <PubSubClient.h>

#ifndef WIFILIBV2_h 
#define WIFILIBV2_h

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_TOPIC      ("AMD")
#define MQTT_SERVER     ("192.168.10.50")       //("192.168.10.50")        // ("192.168.1.2") // ("ec2-52-14-53-218.us-east-2.compute.amazonaws.com")   //  ("192.168.1.7")   //  ("150.165.82.50")
#define MQTT_PORT       (1883)                   //(1883)              //  (995)
#define WIFI_DEBUG      (1)
#define MQTT_MAX_TRANSFER_SIZE 180

#define WIFI_SSID       ("AMD")                 // "brisa-594111";
#define WIFI_PASS       ("amd12345678")         // "gbalklxz";

class WiFiLib
{
    public:
        WiFiLib();            // construtor

        bool begin(void);
        bool mqtt_Init(void);
        void mqtt_loop(void);
        bool mqtt_reconnect(void);
        void mqtt_send(String value);
        String getReceivedMQTT(void);

    private:
        String oledMqttServerStatus;
        const char* ssid     = WIFI_SSID;
        const char* password = WIFI_PASS;
        const char* mqtt_server = MQTT_SERVER;
        const int mqtt_port = MQTT_PORT;
};

#ifdef __cplusplus
}
#endif

#endif  // WIFILIBV2_h