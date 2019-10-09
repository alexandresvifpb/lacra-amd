#include "ESP32LibV2.h"

LinkedList<String> listSendMessage = LinkedList<String>();
uint16_t numberSendMessage;

LinkedList<String> listRecvMessage = LinkedList<String>();
uint16_t numberRecvMessage;

//======================
// Funcoes Public
//======================

// Funcao construtor da classe ESP32Lib
ESP32Lib::ESP32Lib() {}

// Executada automaticamente todas as vezes que o modulo LoRa recebe uma nova mensagem
// Inputs: int com o tamanho do pacote que foi recebido pelo modulo LoRa
// Return: nenhum
void ESP32Lib::addNewSendMessage(String message) {
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
boolean ESP32Lib::avaliableSendMessage(void) {
    // if (numberSendMessage && listSendMessage.size()) return true;
    if (listSendMessage.size()) return true;
    else return false;
    return true;
}

// 
// Inputs: 
// Return: 
String ESP32Lib::getNextSendMessage(void) {
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
String ESP32Lib::encoderJSon(String id, uint8_t type, String payload) {

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
String ESP32Lib::getMacAddress(void) {
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

