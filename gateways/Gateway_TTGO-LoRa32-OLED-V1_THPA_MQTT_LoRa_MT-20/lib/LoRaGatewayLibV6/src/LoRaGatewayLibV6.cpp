#include "LoRaGatewayLibV6.h"

String localAddress;
String packetReceivedLoRa;
int16_t rssi;
float snr;
messageLoRa_t msgReceived;
bool newMsgReceived;

// Funcao construtor
LoRaGatewayLib::LoRaGatewayLib() {}

//====================================
// Funcoes Public
//====================================

// Funcao de inicializacao do modulo LoRa
// Inputs: nenhum
// Return: boolean
boolean LoRaGatewayLib::begin(void) {

    SPI.begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_SS);
    LoRa.setPins(LORA_PIN_SS, LORA_PIN_RST, LORA_PIN_DIO0);// set CS, reset, IRQ pin

    if (!LoRa.begin(LORA_BAND)) {
        Serial.println("LoRa init failed. Check your connections.");
        return 0;
    }

    if (synWord)
        LoRa.setSyncWord(synWord);           // ranges from 0-0xFF, default 0x34, see API docs

    char cLocalAddress[9];
    ultoa(ESP.getEfuseMac(), cLocalAddress, HEX);            // String(long1, HEX)
    localAddress = String(cLocalAddress);

    LoRa.enableCrc();
    LoRa.onReceive(onReceive);
    SetRxMode();

    newMsgReceived = false;

    loraDebugMain = LORA_DEBUG_MAIN;
    loraDebugGetStrRecv = LORA_DEBUG_GETS;
    loraTaskDelayMS = LORA_TASK_DELAY_MS;
    loraDebugTimeSerial = LORA_DEBUG_TIME_SERIAL/loraTaskDelayMS;

    return true;
}

// Funcao para envio de mensagem
// Inputs: nenhum
// Return: boolean
boolean LoRaGatewayLib::send(String strSend) {

    if (loraDebugMain) {
        Serial.print("sendLoRa: ");
        Serial.println(strSend);
    }

    if (LORA_DEBUG_SEND_RECV) {
        Serial.print("LORA_SEND: ");
        Serial.println(strSend);
    }

    SetTxMode();                        // set tx mode
    LoRa.beginPacket();                 // start packet
    LoRa.print(strSend);                // add payload
    LoRa.endPacket();                   // finish packet and send it
    SetRxMode();                        // set rx mode

    return true;
}

// 
// Inputs: nenhum
// Return: boolean
boolean LoRaGatewayLib::isReceived(void) {
    return newMsgReceived;
}

// 
// Inputs: nenhum
// Return: nenhum
messageLoRa_t LoRaGatewayLib::getMsgRecv(void) {
    newMsgReceived = false;
    return msgReceived;
}

// 
// Inputs: nenhum
// Return: String com os dados recebido pelo modulo LoRa
String LoRaGatewayLib::getStrRecv(void) {
    newMsgReceived = false;

    msgReceived = decodeJSON(packetReceivedLoRa);

    String _packetReceived;

    if (msgReceived.id != NULL) {
        _packetReceived.concat("{\"id\":\"");
        _packetReceived.concat(msgReceived.id);
        _packetReceived.concat("\",");
    } else {
        _packetReceived.concat("{\"id\":\"");
        _packetReceived.concat("XXXXXX");
        _packetReceived.concat("\",");
    }

    if (loraDebugGetStrRecv) Serial.print("msgReceived.type: ");
    if (loraDebugGetStrRecv) Serial.println(msgReceived.type);
    
    // if (isDigit((int)msgReceived.type)) {
    if (((int)msgReceived.type > 0)) {

        if (loraDebugGetStrRecv) Serial.println("isDigit");

        _packetReceived.concat("\"type\":");
        _packetReceived.concat(msgReceived.type);
        _packetReceived.concat(",");
    } else {

        if (loraDebugGetStrRecv) Serial.println("Not isDigit");

        _packetReceived.concat("\"type\":");
        _packetReceived.concat("0");
        _packetReceived.concat(",");
    }

    if (msgReceived.payload != NULL) {
        String _payload = msgReceived.payload;
        _payload.replace("[","");
        _payload.replace("]",",");
        _payload.concat(String(rssi, DEC));
        _payload.concat(",");
        _payload.concat(String(snr));

        _packetReceived.concat("\"payload\":[");
        _packetReceived.concat(_payload);
        _packetReceived.concat("]}");
    } else {
        String _payload = packetReceivedLoRa;
        _payload.concat(",");
        _payload.concat(String(rssi, DEC));
        _payload.concat(",");
        _payload.concat(String(snr));

        _packetReceived.concat("\"payload\":\"[");
        _packetReceived.concat(_payload);
        _packetReceived.concat("]\"}");
    }

    return _packetReceived;
}

//====================================
// Funcoes Privates
//====================================

// 
// Inputs: nenhum
// Return: nenhum
void LoRaGatewayLib::onReceive(int packetSize) 
{
    if (packetSize == 0) return;

    String _packetRecv = "";

    while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa
        _packetRecv += (char)LoRa.read();
    }

    rssi = LoRa.packetRssi();
    snr = LoRa.packetSnr();

    if (LORA_DEBUG_SEND_RECV) {
        Serial.print("LORA_RECV: ");
        Serial.println(_packetRecv);
    }

    if (_packetRecv != "") {

        packetReceivedLoRa = _packetRecv;
        newMsgReceived = true;

    }
}

// 
// Inputs: nenhum
// Return: nenhum
void LoRaGatewayLib::SetRxMode(void) {
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}

// 
// Inputs: nenhum
// Return: nenhum
void LoRaGatewayLib::SetTxMode(void) {
    LoRa.idle();                          // set standby mode
    LoRa.enableInvertIQ();                // active invert I and Q signals
}

// Converte um messageLoRa_t em uma estrutura String
// Inputs: messageLoRa_t a ser convertida
// Return: String resultante codificação da messageLoRa_t de entrada
String LoRaGatewayLib::encodeJSON(messageLoRa_t msgSend, uint8_t type) {

    String _strJSON;

    StaticJsonDocument<256> doc;

    JsonObject root = doc.to<JsonObject>();

    root["id"] = msgSend.id;
    root["payload"] = msgSend.payload;

    if (serializeJson(doc, _strJSON) == 0) {
        Serial.println(F("Failed to write to file"));
    }

    return _strJSON;
}

// Converte um String em uma estrutura messageLoRa_t
// Inputs: String a ser convertida
// Return: messageLoRa_t resultante decodificação da String de entrada
messageLoRa_t LoRaGatewayLib::decodeJSON(String strJSONRecv) {

    messageLoRa_t _message;

    StaticJsonDocument<200> doc;

    // strJSONRecv = "{\"id\":\"b93a7d80\",\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\"}";
    DeserializationError error = deserializeJson(doc, strJSONRecv);

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