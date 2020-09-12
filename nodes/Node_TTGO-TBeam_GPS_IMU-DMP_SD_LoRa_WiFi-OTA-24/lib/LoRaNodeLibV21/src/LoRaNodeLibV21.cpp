#include "LoRaNodeLibV21.h"

LinkedList<String> listRecvMessage = LinkedList<String>();
LinkedList<recordFormatLoRa_t> listSendRecord = LinkedList<recordFormatLoRa_t>();
LinkedList<recordFormatLoRa_t> listRecvRecord = LinkedList<recordFormatLoRa_t>();
uint8_t synWord = LORA_SYN_WORD;

LoRaNodeLib::LoRaNodeLib() {}

// LoRa module initialization function
boolean LoRaNodeLib::begin(void) {

    SPI.begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_SS);
    LoRa.setPins(SS, LORA_PIN_RST, LORA_PIN_DIO0);

    if (!LoRa.begin(LORA_BAND))
    {
        Serial.println("LoRa init failed. Check your connections.");
        return false;
    }

    if (synWord)
        LoRa.setSyncWord(synWord);                                          // ranges from 0-0xFF, default 0x34, see API docs

    LoRa.enableCrc();
    LoRa.onReceive(onReceive);
    SetRxMode();

    return true;
}

//
void LoRaNodeLib::run(void) {
    while ( listSendRecord.size() > 0 ) {
        recordFormatLoRa_t record = listSendRecord.remove(0);
        send( encodeJSON(record) );
        delay(100);
    }
}

//
boolean LoRaNodeLib::waitForNextSend(void) {
    if ( millis() >  (tsLoRaDelaySendRecord + lastSendTime) ) {
        lastSendTime = millis();
        return true;
    } else {
        return false;
    }
}

//
boolean LoRaNodeLib::addRecordToSend(recordFormatLoRa_t value) {
    return listSendRecord.add(value);
}

// 
boolean LoRaNodeLib::send(String strSend) {

    bool result = false;

    Serial.print("LORA_SEND: ");
    Serial.println(strSend);

    SetTxMode();                        // set tx mode
    LoRa.beginPacket();                 // start packet
    LoRa.print(strSend);                // add payload
    LoRa.endPacket();                   // finish packet and send it
    SetRxMode();                        // set rx mode

    result = true;
    return result;
}

// 
boolean LoRaNodeLib::isReceived(void) {
    return listRecvMessage.size();
}

// 
String LoRaNodeLib::getStrRecv(void) {
    return listRecvMessage.remove(0);
}

// 
recordFormatLoRa_t LoRaNodeLib::getRecv(void) {
    return listRecvRecord.remove(0);
}

// Automatically executed every time the LoRa module receives a new message
void LoRaNodeLib::onReceive(int packetSize) {
    if (packetSize == 0) return;

    String _packetRecv = "";

    while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa
        _packetRecv += (char)LoRa.read();
    }

    if (_packetRecv != "") {
        listRecvMessage.add(_packetRecv);
    }

}

// Lora sets the module to work in RX mode
void LoRaNodeLib::SetRxMode(void) {
    LoRa.enableInvertIQ();                // active invert I and Q signals
    LoRa.receive();                       // set receive mode
}

// Lora sets the module to work in TX mode
void LoRaNodeLib::SetTxMode(void) {
    LoRa.idle();                          // set standby mode
    LoRa.disableInvertIQ();               // normal mode
}

// Convert a messageLoRa_t to a String structure
String LoRaNodeLib::encodeJSON(recordFormatLoRa_t msgSend) {

    String _strJSON;

    StaticJsonDocument<256> doc;

    JsonObject root = doc.to<JsonObject>();

    root["id"] = msgSend.id;
    root["type"] = msgSend.type;
    root["payload"] = msgSend.payload;

    if (serializeJson(doc, _strJSON) == 0) {
        Serial.println(F("Failed to write to file"));
    }

    return _strJSON;
}

// Converts a String to a recordFormatLoRa_t structure
recordFormatLoRa_t LoRaNodeLib::decodeJSON(String strJSONRecv) {
    recordFormatLoRa_t _message;

    StaticJsonDocument<200> doc;

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