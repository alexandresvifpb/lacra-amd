#include "WiFiLibV4.h"

WiFiClient espClient;
PubSubClient client(espClient);

String strReceivedMQTT;
bool newMsgReceivedWiFi;

void callback(char* topic, byte* message, unsigned int length);
void configModeCallback(WiFiManager *myWiFiManager);
void saveConfigCallback(void);

WiFiLib::WiFiLib() {}

bool WiFiLib::begin(void) {

/*
    //declaração do objeto wifiManager
    WiFiManager wifiManager;
 
    //utilizando esse comando, as configurações são apagadas da memória
    //caso tiver salvo alguma rede para conectar automaticamente, ela é apagada.
    // wifiManager.resetSettings();
 
    //callback para quando entra em modo de configuração AP
    wifiManager.setAPCallback(configModeCallback); 
    //callback para quando se conecta em uma rede, ou seja, quando passa a trabalhar em modo estação
    wifiManager.setSaveConfigCallback(saveConfigCallback); 
 
    //cria uma rede de nome ESP_AP com senha 12345678
    wifiManager.autoConnect("AP_AMD"); 

    newMsgReceivedWiFi = false;

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    // if (WiFi.status() == WL_CONNECTED) return true;
    // return false;
*/

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

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    mqtt_send("Gateway Inicializado");
    
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
    
    client.publish(MQTT_TOPIC, value.c_str());

    if (WIFI_DEBUG) {
        Serial.print("MQTT_SEND: ");
        Serial.println(value);
    }
}

String WiFiLib::getReceivedMQTT(void) {
    newMsgReceivedWiFi = false;
    return strReceivedMQTT;
}

void callback(char* topic, byte* message, unsigned int length) {
    
    if (WIFI_DEBUG) {
        Serial.print("Message arrived on topic: ");
        Serial.print(topic);
        Serial.print(". Message: ");
    }

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

//callback que indica que o ESP entrou no modo AP
void configModeCallback (WiFiManager *myWiFiManager) {  
  Serial.println("Entrou no modo de configuração");
  Serial.println(WiFi.softAPIP()); //imprime o IP do AP
  Serial.println(myWiFiManager->getConfigPortalSSID()); //imprime o SSID criado da rede
 
}

//callback que indica que salvamos uma nova rede para se conectar (modo estação)
void saveConfigCallback () {
  Serial.println("Configuração salva");
}