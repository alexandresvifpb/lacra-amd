#include "WiFiLibV2.h"

WiFiClient espClient;
PubSubClient client(espClient);

String strReceivedMQTT;
bool newMsgReceivedWiFi;

void callback(char* topic, byte* message, unsigned int length);

WiFiLib::WiFiLib() {}

bool WiFiLib::begin(void) {
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

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    newMsgReceivedWiFi = false;

    return true;
}

//===========================================
// MQTT

bool WiFiLib::mqtt_Init(void) {
    Serial.println();
    Serial.print("MQTT Server: ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);

    // if (espClient.connect(mqtt_server, 80)) {
    //     Serial.println("WiFi connected!");
    // }

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    return true;
}

void WiFiLib::mqtt_loop(void) {
    client.loop();
}

bool WiFiLib::mqtt_reconnect(void) {
    // Loop until we're reconnected
    bool _connected = false;
    uint8_t _cont = 12;
    while (!client.connected() && _cont) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP32Client")) {
            Serial.println("connected");
            // Subscribe
            client.subscribe("esp32/output");
            _connected = true;
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            _cont--;
            delay(5000);
        }
    }
    return _connected;
}

void WiFiLib::mqtt_send(String value) {
    if (!client.connected()) {
        mqtt_reconnect();
    }
    if (WIFI_DEBUG) {
        Serial.print("send message MQTT 2: ");
        Serial.println(value);
    }
    client.publish(MQTT_TOPIC, value.c_str());
}

String WiFiLib::getReceivedMQTT(void) {
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
