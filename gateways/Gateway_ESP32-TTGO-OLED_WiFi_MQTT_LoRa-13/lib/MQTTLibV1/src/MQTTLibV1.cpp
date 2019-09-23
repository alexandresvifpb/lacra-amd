#include "MQTTLibV1.h"

WiFiClient espWiFiClient;
PubSubClient mqttClient(espWiFiClient);

String strReceivedMQTT;
bool newMsgReceivedWiFi;

void callback(char* topic, byte* message, unsigned int length);

MQTTLib::MQTTLib() {}

bool MQTTLib::begin(void) {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        isWiFiStatusConnected = true;
    
        Serial.println();
        Serial.println("WiFi connected...");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
    
    newMsgReceivedWiFi = false;

    return true;
}

//===========================================
// MQTT

bool MQTTLib::init(void) {
    Serial.println();
    Serial.print("MQTT Server: ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);

    // if (espWiFiClient.connect(mqtt_server, 80)) {
    //     Serial.println("WiFi connected!");
    // }

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(callback);
    return true;
}

void MQTTLib::loop(void) {
    mqttClient.loop();
}

bool MQTTLib::reconnect(void) {
    // Loop until we're reconnected
    bool _connected = false;
    uint8_t _cont = 12;
    while (!mqttClient.connected() && _cont) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqttClient.connect("ESP32Client")) {
            Serial.println("connected");
            // Subscribe
            mqttClient.subscribe("esp32/output");
            _connected = true;
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            _cont--;
            delay(5000);
        }
    }
    return _connected;
}

void MQTTLib::send(String value) {
    if (!mqttClient.connected()) {
        reconnect();
    }
    if (WIFI_DEBUG) {
        Serial.print("send message MQTT 2: ");
        Serial.println(value);
    }
    mqttClient.publish(MQTT_TOPIC, value.c_str());
}

String MQTTLib::getReceivedMQTT(void) {
    newMsgReceivedWiFi = false;
    return strReceivedMQTT;
}

void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
  
    for (int i = 0; i < length; i++) {
        // Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
//   Serial.println();

//   if (String(topic) == "gateway/1") {
//     Serial.print("Changing output to ");
//     Serial.println(messageTemp);
//   }

    if (messageTemp != "") {
        Serial.println(messageTemp);
        strReceivedMQTT = messageTemp;
        newMsgReceivedWiFi = true;
    }
}

boolean MQTTLib::isWiFiConnected(void) {
    return espWiFiClient.connected();
}