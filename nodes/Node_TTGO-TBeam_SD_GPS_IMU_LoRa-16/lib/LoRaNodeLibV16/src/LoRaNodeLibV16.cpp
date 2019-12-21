#include "LoRaNodeLibV16.h"

LinkedList<messageLoRa_t> sendMessagesList = LinkedList<messageLoRa_t>();
String packetReceivedLoRa;
String macAddress;
String strReceived;
messageLoRa_t msgReceived;
bool newRecvMessages;
uint8_t newSendMessages = 0;
float VBat;

String getChipID(void);

LoRaNodeLib::LoRaNodeLib() {}

// Função de inicialização do modulo LoRa
// Inputs: nenhum
// Return: bool com verdadeira se a inicilizacao foi feita com sucesso e falsa em casa contrario
boolean LoRaNodeLib::begin(void) {

    SPI.begin(LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_SS);
    LoRa.setPins(SS, LORA_PIN_RST, LORA_PIN_DIO0);                          // set CS, reset, IRQ pin

    if (!LoRa.begin(LORA_BAND))
    {
        Serial.println("LoRa init failed. Check your connections.");
        return false;
    }

    if (synWord)
        LoRa.setSyncWord(synWord);                                          // ranges from 0-0xFF, default 0x34, see API docs

    macAddress = getChipID();

    LoRa.enableCrc();
    LoRa.onReceive(onReceive);
    SetRxMode();
    
    newRecvMessages = false;

    loraDebugMain = LORA_DEBUG_MAIN;
    // loraDebugSendRecv = LORA_DEBUG_SEND_RECV;
    loraTaskDelayMS = LORA_TASK_DELAY_MS;
    loraDebugTimeSerial = LORA_DEBUG_TIME_SERIAL/loraTaskDelayMS;

    return true;
}

// 
// Inputs: String
// Return: boolean
boolean LoRaNodeLib::send(String strSend) {

    bool result = false;

    if (loraDebugMain) {
        Serial.print("LORA_SEND: ");
        Serial.println(strSend);
    }

    SetTxMode();                        // set tx mode
    LoRa.beginPacket();                 // start packet
    LoRa.print(strSend);                // add payload
    LoRa.endPacket();                   // finish packet and send it
    SetRxMode();                        // set rx mode

    result = true;
    return result;
}

// 
// Inputs: nenhum
// Return: boolean
boolean LoRaNodeLib::isReceived(void) {
    return newRecvMessages;
}

// 
// Inputs: nenhum
// Return: String
String LoRaNodeLib::getStrRecv(void) {
    newRecvMessages = false;
    return strReceived;
}

// 
// Inputs: nenhum
// Return: messageLoRa_t
messageLoRa_t LoRaNodeLib::getMsgRecv(void) {
    newRecvMessages = false;
    return decodeJSON(packetReceivedLoRa);
}

// 
// Inputs: nenhum
// Return: float
float LoRaNodeLib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}

// 
// Inputs: nenhum
// Return: boolean
boolean LoRaNodeLib::requestSyncTime(void) {

    if (now() > LORA_EPOCH_TIME_2019) return true;

    boolean syncTimeRecv = false;

    String _strCommandSendLoRa = "{\"id\":\"";
    _strCommandSendLoRa += macAddress;
    _strCommandSendLoRa += "\",\"type\":";
    _strCommandSendLoRa += 9;
    _strCommandSendLoRa += ",\"payload\":[";
    _strCommandSendLoRa += "]}";

    send(_strCommandSendLoRa);

    Serial.println(_strCommandSendLoRa);
    delay(100);

    // esp_task_wdt_reset();
    uint16_t _loop = 2000;
    while (!newRecvMessages && _loop) {
        delay(1);
        _loop--;
    }    

    if (newRecvMessages) syncTimeRecv = true;
    
    return syncTimeRecv;
}

// Recupera o MAC do modulo
// Input: nenhum
// Return: nenhum
void LoRaNodeLib::setEpochTime(time_t epochTime) {

    setTime(epochTime);     // Set Time

}

//========================================
// Private
//========================================

// Executada automaticamente todas as vezes que o modulo LoRa recebe uma nova mensagem
// Inputs: int com o tamanho do pacote que foi recebido pelo modulo LoRa
// Return: nenhum
void LoRaNodeLib::onReceive(int packetSize) {
    if (packetSize == 0) return;

    String _packetRecv = "";

    while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa
        _packetRecv += (char)LoRa.read();
    }

    if (LORA_DEBUG_SEND_RECV) {
        Serial.print("LORA_RECV: ");
        Serial.println(_packetRecv);
    }

    if (_packetRecv != "") {

        packetReceivedLoRa = _packetRecv;
        newRecvMessages = true;          //newRecvMessages

    }

}

// Configura o modulo LoRa para trabalhar em modo RX
// Inputs: nenhum
// Return: nenhum
void LoRaNodeLib::SetRxMode(void) {
    LoRa.enableInvertIQ();                // active invert I and Q signals
    LoRa.receive();                       // set receive mode
}

// Configura o modulo LoRa para trabalhar em modo TX
// Input: nenhum
// Return: nenhum
void LoRaNodeLib::SetTxMode(void) {
    LoRa.idle();                          // set standby mode
    LoRa.disableInvertIQ();               // normal mode
}

// Converte um messageLoRa_t em uma estrutura String
// Input: messageLoRa_t a ser convertida type tipo do dados gerado a ser convertido 
// Return: String resultante codificação da messageLoRa_t de entrada
// String LoRaNodeLib::encodeJSON(messageLoRa_t msgSend, uint8_t type) {
String LoRaNodeLib::encodeJSON(messageLoRa_t msgSend) {

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

// Converte um String em uma estrutura messageLoRa_t
// Inputs: String a ser convertida
// Return: messageLoRa_t resultante decodificação da String de entrada
messageLoRa_t LoRaNodeLib::decodeJSON(String strJSONRecv) {
    
    if (LORA_DEBUG_MAIN) {
        Serial.print("decoderJSON.strJSONRecv: ");
        Serial.println(strJSONRecv);
    }

    messageLoRa_t _message;

    StaticJsonDocument<200> doc;
    
    if (LORA_DEBUG_MAIN) {
        Serial.println("decodeJson: #1");
    }

    DeserializationError error = deserializeJson(doc, strJSONRecv);
    
    if (LORA_DEBUG_MAIN) {
        Serial.println("decodeJson: #2");
    }

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return _message;
    }
    
    if (LORA_DEBUG_MAIN) {
        Serial.println("decodeJson: #3");
    }

    JsonObject root = doc.as<JsonObject>();
    
    if (LORA_DEBUG_MAIN) {
        Serial.println("decodeJson: #4");
    }

    String id = root["id"];
    uint8_t type = root["type"];
    String payload = root["payload"];
    
    if (LORA_DEBUG_MAIN) {
        Serial.println("decodeJson: #5");
    }

    _message.id = id;
    _message.type = type;
    _message.payload = payload;

    return _message;

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

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}
