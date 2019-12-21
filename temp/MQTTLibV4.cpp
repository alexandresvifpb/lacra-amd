#include "MQTTLibV4.h"

uint64_t unixTimeNow;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, NTP_UTC_OFFSET * NTP_TIME_ADJUSTMENT, NTP_UPDATE_INTERVAL);    // NTPClient timeClient(ntpUDP, NTP_SERVER, -10800, 60000);

WiFiClient espWiFiClient;
PubSubClient clientMQTT(espWiFiClient);

String strReceivedMQTT;
boolean newMsgReceivedMQTT;
boolean MQTTBrokerConnected = false;
boolean WiFiConnected = false;

// Funcoes privadas
void callback(char* topic, byte* message, unsigned int length);
String getChipID(void);

// Funcao construtora da classe
MQTTLib::MQTTLib() {}

boolean MQTTLib::begin(void) {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    connectWiFi();

    macAddress = getChipID();
    if (mqttDebugBegin) Serial.println(macAddress);

    newMsgReceivedMQTT = false;
    if (checkWiFiConnection()) {

        Serial.println();
        Serial.println("WiFi connected...");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        MQTTBrokerConnected = init();

        if (!MQTTBrokerConnected) {
            Serial.println("Broker MQTT connected...");
            MQTTBrokerConnected = reconnect();
        }

        timeClient.begin();

        mqttDebugMain = MQTT_DEBUG_MAIN;
        mqttDebugBegin = MQTT_DEBUG_BEGIN;
        mqttDebugLoop = MQTT_DEBUG_LOOP;
        wifiDebug = WIFI_DEBUG;
        mqttTaskDelayMS = MQTT_TASK_DELAY_MS;
        recvMqttTaskDelayMS = MQTT_RECV_TASK_DELAY_MS;
        mqttDebugTimeSerial = MQTT_DEBUG_TIME_SERIAL/mqttTaskDelayMS;
        
        return true;
    }

    return false;
}

boolean MQTTLib::init(void) {
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
    if (mqttDebugLoop) Serial.println("---> loop()");

    clientMQTT.loop();      // mantem o cliente MQTT ativo no broker

    // verificar que o time estar ativo e corrigem caso necessario
    uint8_t ltest = NTP_LOOP_UPDATE;
    while(!timeClient.update() && ltest) {
        if (mqttDebugLoop) Serial.println("---> timeClient.update()");

        timeClient.forceUpdate();
        ltest--;

        if (mqttDebugLoop) Serial.println("timeClient.update() --->");
    }

    if (mqttDebugLoop) Serial.println("loop() --->");
}

boolean MQTTLib::reconnect(void) {
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
    if (!MQTTBrokerConnected) {
        MQTTBrokerConnected = reconnect();
        if (!MQTTBrokerConnected) return;
    }

    if (wifiDebug) {
        Serial.print("SEND_MQTT: ");
        Serial.println(value);
    }
    
    clientMQTT.publish(MQTT_TOPIC, value.c_str());
}

String MQTTLib::getReceivedMQTT(void) {
    newMsgReceivedMQTT = false;
    return strReceivedMQTT;
}

boolean MQTTLib::isNewRecvMsgMQTT(void) {
    return newMsgReceivedMQTT;
}

boolean MQTTLib::isConnected(void) {
    return checkWiFiConnection();
}

boolean MQTTLib::checkWiFiConnection(void) {
    if (WiFi.status() == WL_CONNECTED) return true;
    return false;
}

boolean MQTTLib::connectWiFi(void) {
    uint8_t loop = 60;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while ((WiFi.status() != WL_CONNECTED) && (loop > 0)) {
        Serial.print(".");
        delay(500);
        loop--;
    }

    if (checkWiFiConnection()) return false;
    return true;
}

boolean MQTTLib::checkInternet(void) {
    if (!checkWiFiConnection()) {
        if (!connectWiFi()) {
            return false;
        }
    }

    WiFiClient clientWiFi;

    IPAddress adr = IPAddress(172, 217, 3, 110);        // Endereco IP do servidor Google
    clientWiFi.setTimeout(15);                          // Espera por 15 segundos resposta do servidor
    boolean connected = clientWiFi.connect(adr, 80);    // Tenta abrir coneccao com o servidor, retorna true se bem sucedido e false caso contrario
    clientWiFi.stop();                                  // Fecha coneccao

    return connected;
}

uint64_t MQTTLib::getEpochTime(void) {
    return timeClient.getEpochTime();
}

String MQTTLib::getMacAddress(void) {
    return macAddress;
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
        newMsgReceivedMQTT = true;
    }
}

// Recupera o MAC do modulo
// Input: nenhum
// Return: String com parte do MAC (3 bytes) do modulo
String getChipID(void) {
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.

    uint8_t id8 = (uint8_t)(chipID>>24);
    if (id8 > 9) sprintf(chipID_0, "%0X", id8);
    else sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    if (id16 > 9) sprintf(chipID_1, "%0X", id16);
    else sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    if (id32 > 9) sprintf(chipID_2, "%0X", id32);
    else sprintf(chipID_2, "0%0X", id32);

    // sprintf(chipID_0, "%0X", (uint8_t)(chipID>>24));
    // sprintf(chipID_1, "%0X", (uint8_t)(chipID>>32));
    // sprintf(chipID_2, "%0X", (uint8_t)(chipID>>40));

    // Serial.println(chipID_0);
    // Serial.println(chipID_1);
    // Serial.println(chipID_2);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}
