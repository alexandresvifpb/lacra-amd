#include "ESP32NodeLibV18.h"

LinkedList<String> listSendMessage = LinkedList<String>();
uint16_t numberSendMessage;

LinkedList<String> listRecvMessage = LinkedList<String>();
uint16_t numberRecvMessage;

void startOTA(void);
void endOTA(void);
void progressOTA(unsigned int progress, unsigned int total);
void errorOTA(ota_error_t error);

//======================
// Funcoes Public
//======================

// Funcao construtor da classe ESP32Lib
ESP32NodeLib::ESP32NodeLib() {}

// 
// Inputs: Nenhum
// Return: boolean com status da conexao WiFi
boolean ESP32NodeLib::begin(void) {
    //define wifi como station (estação)
    WiFi.mode(WIFI_STA);

    //inicializa wifi
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.println("Begin WiFi ...");
    
    return connectOTA();
}

// 
// Inputs: Nenhum
// Return: boolean com status da conexao WiFi
boolean ESP32NodeLib::connectOTA(void) {

    //enquanto o wifi não for conectado aguarda por WAITCONNECTION
    uint8_t contWiFi = WAITCONNECTION;
    while ((WiFi.waitForConnectResult() != WL_CONNECTED) && contWiFi) {
        //caso falha da conexão, reinicia wifi
        // Serial.print("Connection Failed! Rebooting...");
        Serial.print(".");
        delay(1000);
        contWiFi--;
        // ESP.restart();
    }

    if (WL_CONNECTED) {

        // A porta fica default como 3232
        // ArduinoOTA.setPort(3232);

        // Define o hostname (opcional)
        hostname = HOSTNAME + getMacAddress();
        ArduinoOTA.setHostname(hostname.c_str());

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
        Serial.println(hostname);

        connectedOTA = true;

    } 

    return connectedOTA;

}

// Executada automaticamente todas as vezes que o modulo LoRa recebe uma nova mensagem
// Inputs: int com o tamanho do pacote que foi recebido pelo modulo LoRa
// Return: nenhum
void ESP32NodeLib::addNewSendMessage(String message) {
    if (esp32DebugAdd) Serial.print("addNewSendMessage: ");
    if (esp32DebugAdd) Serial.println(message);
    listSendMessage.add(message);
    if (esp32DebugAdd) Serial.print("listSendMessage.size: ");
    if (esp32DebugAdd) Serial.println(listSendMessage.size());
    numberSendMessage++;
    if (esp32DebugAdd) Serial.print("numberSendMessage: ");
    if (esp32DebugAdd) Serial.println(numberSendMessage);
}

// 
// Inputs: 
// Return: 
boolean ESP32NodeLib::avaliableSendMessage(void) {
    // if (numberSendMessage && listSendMessage.size()) return true;
    if (listSendMessage.size()) return true;
    else return false;
    return true;
}

// 
// Inputs: 
// Return: 
String ESP32NodeLib::getNextSendMessage(void) {
    String _sendMessage;
    // if (numberSendMessage && listSendMessage.size()) {
    if (listSendMessage.size()) {
        if (esp32DebugGet) Serial.println("listSendMessage.size(): ");
        _sendMessage = listSendMessage.remove(0);
        numberSendMessage--;
    }

    if (esp32DebugGet) Serial.print("getNextSendMessage: ");
    if (esp32DebugGet) Serial.println(_sendMessage);

    return _sendMessage;
}

// Funcao para criar uma string codificada em JSON
// Inputs:  String id - endereco do dispositivo
//          uint8_t type - tipo do dados a ser salvo
//          String payload - dados a ser salvo
// Return: String retorna uma string com os dados codificado em JSON
String ESP32NodeLib::encoderJSon(String id, uint8_t type, String payload) {

    String _strJSON;

    StaticJsonDocument<256> doc;

    JsonObject root = doc.to<JsonObject>();

    root["id"] = id;
    root["type"] = type;
    root["payload"] = payload;

    if (serializeJson(doc, _strJSON) == 0) {
        Serial.println(F("Failed to write to file"));
    }

    return _strJSON;

}

// Recupera o MAC do modulo
// Input: nenhum
// Return: String com parte do MAC (3 bytes) do modulo
String ESP32NodeLib::getMacAddress(void) {
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

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float ESP32NodeLib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}

// Funções de exibição dos estágios de upload (start, progress, end e error) do ArduinoOTA
// Inputs: void
// Return: void
// void ESP32NodeLib::startOTA(void) {
void startOTA(void) {
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
void endOTA(void) {
    Serial.println("\nEnd");
}

// Envia para serial a progresso em porcentagem
// Inputs: int, int valor do progresso e total
// Return: void
// void ESP32NodeLib::progressOTA(unsigned int progress, unsigned int total) {
void progressOTA(unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
}

// Envia para serial mensagens caso aconteça algum erro, exibe especificamente o tipo do erro
// Inputs: ota_error_t gerado na lib ArduinoOTA
// Return: void
// void ESP32NodeLib::errorOTA(ota_error_t error) {
void errorOTA(ota_error_t error) {
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