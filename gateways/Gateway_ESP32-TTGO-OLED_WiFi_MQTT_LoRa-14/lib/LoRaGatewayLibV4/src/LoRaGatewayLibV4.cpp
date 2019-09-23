#include "LoRaGatewayLibV4.h"

String localAddress;
String packetReceivedLoRa;
int16_t rssi;
float snr;
messageLoRa_t msgReceived;
// bool receivedStr;
bool newMsgReceived;

LoRaGatewayLib::LoRaGatewayLib() {}

bool LoRaGatewayLib::begin(void) {

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);// set CS, reset, IRQ pin

    if (!LoRa.begin(BAND))
    {
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

    return true;
}

bool LoRaGatewayLib::send(String strSend) {

    if (DEBUG_LORALIB) {
        Serial.print("sendLoRa: ");
        Serial.println(strSend);
    }

    SetTxMode();                        // set tx mode
    LoRa.beginPacket();                 // start packet
    LoRa.print(strSend);                // add payload
    LoRa.endPacket();                   // finish packet and send it
    SetRxMode();                        // set rx mode

    return true;
}

bool LoRaGatewayLib::isReceived(void) {
    return newMsgReceived;
}

messageLoRa_t LoRaGatewayLib::getMsgRecv(void) {
    newMsgReceived = false;
    return msgReceived;
}

String LoRaGatewayLib::getStrRecv(void) {
    newMsgReceived = false;

    msgReceived = decodeJSON(packetReceivedLoRa);

    String _packetReceived = "[";

    if (msgReceived.payload == NULL) {
        String _payload = msgReceived.payload;

        _packetReceived.concat(msgReceived.id);
        _packetReceived.concat(",");
        _packetReceived.concat(msgReceived.type);
        _payload.replace("[",",");
        _payload.replace("]",",");
        _packetReceived.concat(_payload);
        _packetReceived.concat(String(rssi, DEC));
        _packetReceived.concat(",");
        _packetReceived.concat(String(snr));
    } else {
        packetReceivedLoRa.replace("[","");
        packetReceivedLoRa.replace("]","");
        _packetReceived.concat(packetReceivedLoRa);
        _packetReceived.concat(",");
        _packetReceived.concat(String(rssi, DEC));
        _packetReceived.concat(",");
        _packetReceived.concat(String(snr));
    }

    _packetReceived.concat("]");

/*
    if (DEBUG_LORALIB) {

        Serial.println("------------");

        Serial.print("packetReceivedLoRa: ");
        Serial.println(packetReceivedLoRa);

        Serial.print("msgReceived.id: ");
        Serial.println(msgReceived.id);

        Serial.print("msgReceived.type: ");
        Serial.println(msgReceived.type);

        Serial.print("msgReceived.payload: ");
        Serial.println(msgReceived.payload);

        Serial.print("_packetReceived: ");
        Serial.println(_packetReceived);

        Serial.println("------------");
    }
*/

    return _packetReceived;
}

/*
void cbk(int packetSize) {
    packet ="";
    packSize = String(packetSize,DEC);
    for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
    // rssi = " RSSI " + String(LoRa.packetRssi(), DEC) ;
    rssi = LoRa.packetRssi();
    snr = LoRa.packetSnr();

    // loraData();
    Serial.println(parseData(packet, rssi, snr));
}

String parseData(String textRcv, int _rssi, float _snr) {
    textRcv.replace("]",",");
    textRcv.concat(String(_rssi, DEC));
    textRcv.concat(",");
    textRcv.concat(String(_snr));
    textRcv.concat("]");
    return textRcv;
}
*/

//====================================
// private
void LoRaGatewayLib::onReceive(int packetSize) 
{
    if (packetSize == 0) return;

    String _packetRecv = "";
    // int16_t _rssi;
    // float _snr;

    while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa
        _packetRecv += (char)LoRa.read();
    }

    rssi = LoRa.packetRssi();
    snr = LoRa.packetSnr();

    if (DEBUG_LORALIB) {
        Serial.print("RCEV_LORA: ");
        Serial.println(_packetRecv);
    }

/*
    if (_packetRecv != "") {

        msgReceived = decodeJSON(_packetRecv);

        msgReceived.rssi = _rssi;
        msgReceived.snr = _snr;

        // strReceived = _strRecv;
        // msgReceived = decodeJSON(strReceived);
        newMsgReceived = true;

    }
    */

    if (_packetRecv != "") {

        packetReceivedLoRa = _packetRecv;
        // msgReceived.rssi = _rssi;
        // msgReceived.snr = _snr;
        // msgReceived = decodeJSON(strReceived);
        newMsgReceived = true;

        // if (newMsgReceived) Serial.println("mensagem recebida");
        // else Serial.println("Sem mensagens");

    }
}

void LoRaGatewayLib::SetRxMode(void) {
    LoRa.disableInvertIQ();               // normal mode
    LoRa.receive();                       // set receive mode
}

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

/*
    if (DEBUG_LORALIB) {
        Serial.print("strJSONRecv 2: ");
        Serial.println(strJSONRecv);
    }
*/

    messageLoRa_t _message;

    StaticJsonDocument<200> doc;

    // {"id":"b93a7d80","type":0,"payload":"[946588835,-7.225489,-35.889164,539.10,0.50,6,193]"}
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\"}";
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"type\":\"0\"}";
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"type\":0}";
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"type\":0,\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\"}";
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"type\":\"0\",\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\"}";
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\",\"type\":0}";
    DeserializationError error = deserializeJson(doc, strJSONRecv);
    // strJSONRecv = "{\"id\":\"b93a7d80\",\"payload\":\"[946588590,-7.225331,-35.889313,526.20,0.74,6,193]\"}";

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return _message;
    }

/*
    if (DEBUG_LORALIB) {
        Serial.println("Deserialization ok!");
    }
*/

    JsonObject root = doc.as<JsonObject>();

    String id = root["id"];
    uint8_t type = root["type"];
    String payload = root["payload"];

    _message.id = id;
    _message.type = type;
    _message.payload = payload;

    return _message;

}