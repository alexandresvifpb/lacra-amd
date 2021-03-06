#include "MQTTLibV1.h"

WiFiClient espWiFiClient;
PubSubClient clientMQTT(espWiFiClient);

String strReceivedMQTT;
bool newMsgReceivedWiFi;
boolean MQTTBrokerConnected = false;
boolean WiFiConnected = false;

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

    newMsgReceivedWiFi = false;
    // WiFiConnected = espWiFiClient.connected();

    if (WiFi.status() == WL_CONNECTED) {
    // if (WiFiConnected) {

        Serial.println();
        Serial.println("WiFi connected...");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        MQTTBrokerConnected = init();

        if (!MQTTBrokerConnected) {
            Serial.println("Broker MQTT connected...");
            MQTTBrokerConnected = reconnect();
        }

        return true;
    }

    return false;
}

bool MQTTLib::init(void) {
    // if (isWiFiConnected()) {
    if (WiFi.status() == WL_CONNECTED) {

        Serial.println();
        Serial.print("MQTT Server: ");
        Serial.print(mqtt_server);
        Serial.print(":");
        Serial.println(mqtt_port);

        clientMQTT.setServer(mqtt_server, mqtt_port);
        clientMQTT.setCallback(callback);

        send("Gateway Inicializado");

        return true;
    }
    return false;
}

void MQTTLib::loop(void) {
    clientMQTT.loop();
}

bool MQTTLib::reconnect(void) {
    // Loop until we're reconnected
    bool _connected = false;
    uint8_t _cont = 12;
    while (!clientMQTT.connected() && _cont) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (clientMQTT.connect("ESP32Client")) {
            Serial.println("connected");
            // Subscribe
            clientMQTT.subscribe("esp32/output");
            _connected = true;
        } else {
            Serial.print("failed, rc=");
            Serial.print(clientMQTT.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            _cont--;
            delay(5000);
        }
    }
    return _connected;
}

void MQTTLib::send(String value) {
    // if (!clientMQTT.connected()) {
    if (!MQTTBrokerConnected) {
        MQTTBrokerConnected = reconnect();
        if (!MQTTBrokerConnected) return;
    }

    if (WIFI_DEBUG) {
        Serial.print("SEND_MQTT: ");
        Serial.println(value);
    }
    
    clientMQTT.publish(MQTT_TOPIC, value.c_str());
}

String MQTTLib::getReceivedMQTT(void) {
    newMsgReceivedWiFi = false;
    return strReceivedMQTT;
}

bool MQTTLib::isConnected(void) {
    return MQTTBrokerConnected;
}

void callback(char* topic, byte* message, unsigned int length) {
    // Serial.print("Message arrived on topic: ");
    // Serial.print(topic);
    // Serial.print(". Message: ");

    Serial.print("RCEV_MQTT: ");
    String messageTemp;
  
    for (int i = 0; i < length; i++) {
        // Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }

    if (messageTemp != "") {
        Serial.println(messageTemp);
        strReceivedMQTT = messageTemp;
        newMsgReceivedWiFi = true;
    }
}