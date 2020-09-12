#include "WiFiGatewayLibV1.h"

void startOTA(void);
void endOTA(void);
void progressOTA(unsigned int progress, unsigned int total);
void errorOTA(ota_error_t error);
void wifiEventConnected(WiFiEvent_t event, WiFiEventInfo_t info);

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER);    // NTPClient timeClient(ntpUDP, NTP_SERVER, -10800, 60000);

// WEB SERVER
const uint serverPort = WEB_SERVER_PORT;
WiFiServer webServer(serverPort);
WiFiClient remoteClient;

long tsDelaySTA, tsDelayAP, currentTimeSTA, currentTimeAP;

// Construtor da classe WiFiGatewayLib
WiFiGatewayLib::WiFiGatewayLib() {}

//========================

uint64_t unixTimeNow;

WiFiClient espWiFiClient;
PubSubClient clientMQTT(espWiFiClient);

String strReceivedMQTT;
boolean newMsgReceivedMQTT;
boolean MQTTBrokerConnected = false;
boolean WiFiConnected = false;

// Funcoes privadas
void callback(char* topic, byte* message, unsigned int length);
// String getChipID(void);

// Funcao construtora da classe
MQTTLib::MQTTLib() {}

//========================

//======================
// Funcoes Public
//======================

// Inicializa a classe
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiGatewayLib::begin(void)
{
    Serial.println("Begin WiFi ...");

    tsDelaySTA = STA_DELAY_MS;
    currentTimeSTA = -2*tsDelaySTA;
    tsDelayAP = AP_DELAY_MS;
    currentTimeAP = -2*tsDelayAP;

    wifi_apssid = "Node-" + getMacAddress();

    return true;
}

// 
// Inputs: void
// Return: uint8_t
uint8_t WiFiGatewayLib::listenForClientsWS(void)
{
    // Serial.print(__FUNCTION__);
    // Serial.println(__LINE__);
    if ( webServer.hasClient() )
    {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        if ( remoteClient.connected() )
        {
            Serial.println("Connection rejected");
            webServer.available().stop();

            return 2;
        }
        else
        {
            Serial.println("Connection accepted");
            remoteClient = webServer.available();
            
            // isConnectedClientAP = true;
            isActiveServiceWS = true;

            return 1;
        }
        // return true;
    }
    return 0;
}

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
uint8_t WiFiGatewayLib::availableClientWS(void)
{
    /*
    // Serial.print(__FUNCTION__);
    // Serial.println(__LINE__);
    if ( webServer.hasClient() )
    {
        // If we are already connected to another computer, 
        // then reject the new connection. Otherwise accept
        // the connection. 
        // Serial.print(__FUNCTION__);
        // Serial.println(__LINE__);
        if ( remoteClient.connected() )
        {
            Serial.println("Connection rejected");
            webServer.available().stop();

            return 2;
        }
        else
        {
            Serial.println("Connection accepted");
            remoteClient = webServer.available();

            return 1;
        }
        // return true;
    }
    else
    {
        Serial.print("remoteClient.connected(): ");
        Serial.println(remoteClient.connected());

        return remoteClient.connected();
    }
    
    // return 0;
    */

    // Serial.print("remoteClient.connected(): ");
    // Serial.println(remoteClient.connected());

    uint8_t _result = remoteClient.connected();
    if ( _result > 0)
    {
        isConnectedClientAP = true;
        return _result;
    }
    else
    {
        isConnectedClientAP = false;
        isActiveServiceWS = false;
        return _result;
    }
    

    // return remoteClient.connected();

}

// Verifica se o cliente esta conectado e enviou dados
// Inputs: void
// Return: int buffer de dados recebido do cliente
uint8_t WiFiGatewayLib::availableRecvDataClientWS(void)
{
    if ( remoteClient.connected() )
    {
        return remoteClient.available();
    }
    else
    {
        // isActiveServiceWS = false;
        closeClientWS();
        return 0;
    }
}

// Fecha o socket
// Inputs: void
// Return: void
void WiFiGatewayLib::closeClientWS(void)
{
    // Serial.println("closeWebSocket() #1");
    remoteClient.stop();
}

// Ler os dados chegando pelo socket
// Inputs: void
// Return: String com o valor recebido pelo socket
String WiFiGatewayLib::readClientWS(void)
{
    if ( remoteClient )
    {
        return remoteClient.readStringUntil('\n');
    }
    return "";
}

// Escreve na porta do socket
// Inputs: String com o valor a ser enviado pelo socket
// Return: void
void WiFiGatewayLib::printClientWS(String strValue)
{

    // Serial.print("isActiveServiceWS: ");
    // Serial.println(isActiveServiceWS);

    if ( isActiveServiceWS )
    {
        remoteClient.println(strValue.c_str());
    }

    // if ( remoteClient )
    // {
    //     remoteClient.println(strValue.c_str());
    // }
}

// Recupera o MAC do modulo
// Input: nenhum
// Return: String - string com parte do MAC (3 bytes) do modulo
String WiFiGatewayLib::getMacAddress(void) 
{
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    // Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    // Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.

    uint8_t id8 = (uint8_t)(chipID>>24);
    if (id8 > 9) sprintf(chipID_0, "%0X", id8);
    else sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    if (id16 > 9) sprintf(chipID_1, "%0X", id16);
    else sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    if (id32 > 9) sprintf(chipID_2, "%0X", id32);
    else sprintf(chipID_2, "0%0X", id32);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// Inicializa o modulo WiFi do ESP32 no modo STA
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiGatewayLib::initSTA(void)
{
    // Serial.println(__LINE__);
    if ( (millis() - currentTimeSTA) > tsDelaySTA )
    {
        //define wifi como AP + station (estação)
        WiFi.mode(WIFI_STA);

        // Inicializa o objeto wifi passando o ssid e pass
        WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());

        // Verifica se a conexao wifi foi estabelicida, caso ela ainda nao tenha ocorrida
        // entra no loop e aguarda por 60 segundos a resposta do roteador ao pedido de
        // conexao.
        Serial.println("Wait connecting to WiFi...");
        uint8_t _contWhile = 30;
        while ( (WiFi.status() != WL_CONNECTED) && (_contWhile) )
        {
            Serial.print("WIFI_TRY #");
            Serial.println(_contWhile);
            WiFi.reconnect();
            _contWhile--;
            delay(1000);
        }

        webServer.begin(serverPort);

        currentTimeSTA = millis();
    }

    if ( WiFi.status() == WL_CONNECTED )
    {
        connectedModeSTA = true;
        return connectedModeSTA;
    }
    else
    {
        WiFi.mode(WIFI_OFF);
        connectedModeSTA = false;
        return connectedModeSTA;
    }
}

// retorna o status da conexao STA
// Inputs: void
// Return: boolean - TRUE se ativa
boolean WiFiGatewayLib::isConnectedSTA(void)
{

    if ( WiFi.status() == WL_CONNECTED )
    {
        connectedModeSTA = true;
    }
    else
    {
        connectedModeSTA = false;
    }

    return connectedModeSTA;
}

// Coloca o WiFi em off
// Inputs: void
// Return: void
void WiFiGatewayLib::disconnectSTA(void)
{
    webServer.stopAll();

    WiFi.mode(WIFI_OFF);
    connectedModeSTA = false;
}

// Informa se a conexao com a rede wifi foi estabelicida
// Inputs: void
// Return: boolean - TRUE rede wifi conectada
boolean WiFiGatewayLib::activeOTA(void)
{
    if ( connectedModeSTA ) {
        // A porta fica default como 3232
        // ArduinoOTA.setPort(3232);

        // Define o hostname (opcional)
        String _hostname = HOSTNAME + getMacAddress();
        ArduinoOTA.setHostname(_hostname.c_str());

        // Define a senha (opcional)
        //   ArduinoOTA.setPassword("amd1234");

        // É possível definir uma criptografia hash md5 para a senha usando a função "setPasswordHash"
        // Exemplo de MD5 para senha "admin" = 21232f297a57a5a743894a0e4a801fc3
        // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

        //define o que será executado quando o ArduinoOTA iniciar
        ArduinoOTA.onStart(startOTA); //startOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA terminar
        ArduinoOTA.onEnd(endOTA); //endOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA estiver gravando
        ArduinoOTA.onProgress(progressOTA); //progressOTA é uma função criada para simplificar o código 

        //define o que será executado quando o ArduinoOTA encontrar um erro
        ArduinoOTA.onError(errorOTA);//errorOTA é uma função criada para simplificar o código 
  
        //inicializa ArduinoOTA
        ArduinoOTA.begin();

        //exibe pronto e o ip utilizado pelo ESP
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Hostname: ");
        Serial.println(_hostname);

        return true;
    }
    else
    {
        return false;
    }
}

// Inicializa o servico NTP
// Inputs: void
// Return: boolean - TRUE se o cliente NTP foi inicilizado com sucesso
boolean WiFiGatewayLib::bootServiceNTP(void)
{
    if ( WiFi.status() == WL_CONNECTED )
    {
        timeClient.begin();

        // Set offset time in seconds to adjust for your timezone, for example:
        // GMT 1 = 3600
        // GMT 0 = 0
        // GMT -1 = -3600
        // GMT -3 = -10800      (Recife, Brasil)
        timeClient.setTimeOffset( timeOffsetNTP );

        timeClient.setUpdateInterval( updateIntervalNTP );

        timeClient.forceUpdate();

        return true;
        // connectedNTP = true;
        // return connectedNTP;
    }

    return false;
    // connectedNTP = false;
    // return connectedNTP;
}

// Inicializa o servico NTP
// Inputs: void
// Return: boolean - TRUE se o cliente NTP foi inicilizado com sucesso
long WiFiGatewayLib::getUnixTimeNTP(void)
{
    return timeClient.getEpochTime();
}

// Inicializa o modulo WiFi do ESP32 no modo AP
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean WiFiGatewayLib::initAP(void)
{
    // Serial.println(__LINE__);
    if ( (millis() - currentTimeAP) > tsDelayAP )
    {
        // Serial.println(__LINE__);

        //define wifi como AP + station (estação)
        WiFi.mode(WIFI_AP);

        // Inicializa o objeto wifi passando o ssid e pass
        WiFi.softAP(wifi_apssid.c_str(), wifi_appass.c_str());

        Serial.print("WiFi.softAPgetStationNum()");
        Serial.println(WiFi.softAPgetStationNum());

        // Verifica se a conexao wifi foi estabelicida, caso ela ainda nao tenha ocorrida
        // entra no loop e aguarda por 60 segundos a resposta do roteador ao pedido de
        // conexao.
        Serial.println("Wait connecting on AP...");
        uint8_t _contWhile = 30;
        while ( (WiFi.softAPgetStationNum() ==  0) && (_contWhile) )
        {
            // Serial.println(__LINE__);
            Serial.print("AP_TRY #");
            Serial.println(_contWhile);
            // WiFi.reconnect();
            _contWhile--;
            delay(1000);
        }

        webServer.begin(serverPort);

        currentTimeAP = millis();
    }

    if ( WiFi.softAPgetStationNum() > 0 )
    {
        // Serial.println(__LINE__);
        connectedModeAP = true;
        return connectedModeAP;
    }
    else
    {
        // Serial.println(__LINE__);
        WiFi.mode(WIFI_OFF);
        connectedModeAP = false;
        return connectedModeAP;
    }
}

// retorna o status da conexao AP
// Inputs: void
// Return: boolean - TRUE se ativa
boolean WiFiGatewayLib::isConnectedAP(void)
{
    if ( (WiFi.softAPgetStationNum() >  0) )
    {
        connectedModeAP = true;
    }
    else
    {
        connectedModeAP = false;
    }
    
    return connectedModeAP;
}

// Coloca o WiFi em off
// Inputs: void
// Return: void
void WiFiGatewayLib::disconnectAP(void)
{
    webServer.stopAll();
    
    WiFi.mode(WIFI_OFF);
    connectedModeAP = false;
}

// Funções de exibição dos estágios de upload (start, progress, end e error) do ArduinoOTA
// Inputs: void
// Return: void
// void ESP32NodeLib::startOTA(void) {
void startOTA(void) 
{
    String type;
   
    //caso a atualização esteja sendo gravada na memória flash externa, então informa "flash"
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "flash";
    else  //caso a atualização seja feita pela memória interna (file system), então informa "filesystem"
      type = "filesystem"; // U_SPIFFS

    //exibe mensagem junto ao tipo de gravação
    Serial.println("Start updating " + type);
}

// Enviar para serial a mensagem de saida
// Inputs: void
// Return: void
// void ESP32NodeLib::endOTA(void) {
void endOTA(void)
{
    Serial.println("\nEnd");
}

// Envia para serial a progresso em porcentagem
// Inputs: int, int valor do progresso e total
// Return: void
// void ESP32NodeLib::progressOTA(unsigned int progress, unsigned int total) {
void progressOTA(unsigned int progress, unsigned int total) 
{
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
}

// Envia para serial mensagens caso aconteça algum erro, exibe especificamente o tipo do erro
// Inputs: ota_error_t gerado na lib ArduinoOTA
// Return: void
// void ESP32NodeLib::errorOTA(ota_error_t error) {
void errorOTA(ota_error_t error) 
{
      Serial.printf("Error[%u]: ", error);
      
      if (error == OTA_AUTH_ERROR) 
        Serial.println("Auth Failed");
      else
      if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else 
      if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else
      if (error == OTA_RECEIVE_ERROR) 
        Serial.println("Receive Failed");
      else 
      if (error == OTA_END_ERROR)
        Serial.println("End Failed");
}

//========================================================
// classe mqtt
//========================================================


boolean MQTTLib::bootMQTT(void) {
    // connectWiFi();
    WiFiGatewayLib.initSTA();

    // macAddress = getChipID();
    // if (mqttDebugBegin) Serial.println(macAddress);

    newMsgReceivedMQTT = false;
    // if (checkWiFiConnection()) {
    if (checkWiFiConnectionMQTT()) {

        Serial.println();
        Serial.println("WiFi connected...");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        MQTTBrokerConnected = initSTA();

        if (!MQTTBrokerConnected) {
            Serial.println("Broker MQTT connected...");
            // MQTTBrokerConnected = reconnect();
            MQTTBrokerConnected = reconnectMQTT();
        }

        timeClient.begin();

        while( !timeClient.update() )
        {
            timeClient.forceUpdate();
        }

        mqttDebugMain = MQTT_DEBUG_MAIN;
        mqttDebugBegin = MQTT_DEBUG_BEGIN;
        mqttDebugLoop = MQTT_DEBUG_LOOP;
        // wifiDebug = WIFI_DEBUG;
        mqttTaskDelayMS = MQTT_TASK_DELAY_MS;
        recvMqttTaskDelayMS = MQTT_RECV_TASK_DELAY_MS;
        mqttDebugTimeSerial = MQTT_DEBUG_TIME_SERIAL/mqttTaskDelayMS;
        
        return true;
    }

    return false;
}

boolean MQTTLib::initMQTT(void) {
    if (WiFi.status() == WL_CONNECTED) {

        Serial.println();
        Serial.print("MQTT Server: ");
        Serial.print(mqtt_server);
        Serial.print(":");
        Serial.println(mqtt_port);

        clientMQTT.setServer(mqtt_server, mqtt_port);
        clientMQTT.setCallback(callback);

        // send("Gateway Inicializado");
        sendMQTT("Gateway Inicializado");

        return true;
    }
    return false;
}

void MQTTLib::loopMQTT(void) {
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

boolean MQTTLib::reconnectMQTT(void) {
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

void MQTTLib::sendMQTT(String value) {
    if (!MQTTBrokerConnected) {
        // MQTTBrokerConnected = reconnect();
        MQTTBrokerConnected = reconnectMQTT();
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

boolean MQTTLib::checkWiFiConnectionMQTT(void) {
    if (WiFi.status() == WL_CONNECTED) return true;
    return false;
}

boolean MQTTLib::checkInternetMQTT(void) {
    // if (!checkWiFiConnection()) {
        // if (!connectWiFi()) {
    if (!checkWiFiConnectionMQTT()) {
        if (!WiFiGatewayLib.initSTA()) {
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

uint64_t MQTTLib::getEpochTimeMQTT(void) {
    return timeClient.getEpochTime();
}

// Funcao para criar uma string codificada em JSON
// Inputs:  String id - endereco do dispositivo
//          uint8_t type - tipo do dados a ser salvo
//          String payload - dados a ser salvo
// Return: String retorna uma string com os dados codificado em JSON
messageMQTT_t MQTTLib::decoderStrJSonInObjectMsgMQTT(String strJson) {

    messageMQTT_t _message;

    StaticJsonDocument<200> doc;
    
    DeserializationError error = deserializeJson(doc, strJson);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return _message;
    }

    JsonObject root = doc.as<JsonObject>();

    String id = root["id"];
    uint8_t type = root["type"];
    String payload = root["payload"];

    _message.id = id;
    _message.type = type;
    _message.payload = payload;

    return _message;

}


//Private
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
