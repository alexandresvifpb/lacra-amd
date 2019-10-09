#include "LoRaNodeLibV3.h"

// LinkedList<String> sendMessagesList = LinkedList<String>();
LinkedList<messageLoRa_t> sendMessagesList = LinkedList<messageLoRa_t>();
String localAddress;
String strReceived;
messageLoRa_t msgReceived;
bool newRecvMessages;
uint8_t newSendMessages = 0;
float VBat;

LoRaNodeLib::LoRaNodeLib() {}

// Função de inicialização do modulo LoRa
// Inputs: nenhum
// Return: bool com verdadeira se a inicilizacao foi feita com sucesso e falsa em casa contrario
bool LoRaNodeLib::begin(void) {

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
    LoRa.setPins(SS, RST_PIN, DIO0_PIN);                    // set CS, reset, IRQ pin

    if (!LoRa.begin(BAND))
    {
        Serial.println("LoRa init failed. Check your connections.");
        return false;
    }

    if (synWord)
        LoRa.setSyncWord(synWord);                          // ranges from 0-0xFF, default 0x34, see API docs

    // char cLocalAddress[9];
    // ultoa(ESP.getEfuseMac(), cLocalAddress, HEX);            // String(long1, HEX)
    // localAddress = String(cLocalAddress);

    // uint64_t macAddress = ESP.getEfuseMac();
    // uint64_t macAddressTrunc = macAddress << 40;
    // uint64_t chipID = macAddressTrunc >> 40;
    // char cLocalAddress[7];
    // ultoa(chipID, cLocalAddress, HEX);            // String(long1, HEX)
    // localAddress = String(cLocalAddress);

    // char cLocalAddress[8];

    // localAddress = String(chipID);

    // uint8_t id[8];
    // uint64_t chipID = ESP.getEfuseMac();
    // id[0] = 0;
    // id[1] = 0;
    // id[2] = chipID;
    // id[3] = chipID >> 8;
    // id[4] = chipID >> 16;
    // id[5] = chipID >> 24;
    // id[6] = chipID >> 32;
    // id[7] = chipID >> 40;

    // for (int i = 0; i < 8; i++) {
    //     // ultoa(id[i], cLocalAddress[i], HEX);            // String(long1, HEX)
    //     Serial.print(id[i], HEX);
    // }
    // Serial.println();
    // localAddress = String(cLocalAddress);

    char chipID_0[3];
    char chipID_1[3];
    char chipID_2[3];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.
    sprintf(chipID_0, "%0X", (uint8_t)(chipID>>24));
    sprintf(chipID_1, "%0X", (uint8_t)(chipID>>32));
    sprintf(chipID_2, "%0X", (uint8_t)(chipID>>40));

    localAddress = String(chipID_0) + String(chipID_1) + String(chipID_2);
    Serial.print("Local address: ");
    Serial.println(localAddress);

    LoRa.enableCrc();
    LoRa.onReceive(onReceive);
    SetRxMode();
    
    newRecvMessages = false;

    return true;
}

void LoRaNodeLib::run(void) {
    // Serial.print(".");
    // Serial.print(newSendMessages);
    // Serial.print(sendMessagesList.size());
    uint8_t _cont = 2;
    while (newSendMessages && sendMessagesList.size() && _cont) {
        _cont++;
        // String _strMassege = sendMessagesList.remove(0);
        String _strMassege = encodeJSON(sendMessagesList.remove(0));
        // Serial.println(sendMessagesList.size());
        send(_strMassege);
        newSendMessages--;
        delay(1);
    }
    delay(1);
}

// Executa rotina para pegar a primeira mensagem add na lista de msgSendList, converto-a em uma String, e envia-la pelo modulo LoRa
// Executa rotina para enviar uma String pelo modulo LoRa
// Inputs: nenhum
// Return: nenhum
bool LoRaNodeLib::send(String strSend) {

    bool result = false;

    // messageLoRa_t _msgSend;
    // _msgSend.id = localAddress;
    // _msgSend.payload = strSend;

    // String _strSendJSON = encodeJSON(_msgSend);

    if (DEBUG_LORAT) {
        Serial.print("strSendJSON: ");
        // Serial.println(_strSendJSON);
        Serial.println(strSend);
    }

    SetTxMode();                        // set tx mode
    LoRa.beginPacket();                 // start packet
    // LoRa.print(_strSendJSON);                // add payload
    LoRa.print(strSend);                // add payload
    LoRa.endPacket();                   // finish packet and send it
    SetRxMode();                        // set rx mode

    result = true;
    return result;
}

bool LoRaNodeLib::isReceived(void) {
    return newRecvMessages;
}

String LoRaNodeLib::getStrRecv(void) {
    newRecvMessages = false;
    return strReceived;
}

messageLoRa_t LoRaNodeLib::getMsgRecv(void) {
    newRecvMessages = false;
    return msgReceived;
}
        
// pega o valor do endereço MAC local do ESP32
// Inputs: nenhum
// Return: String com o valor do endereço MAC local
String LoRaNodeLib::getLocalAddress(void) {
    return localAddress;
}

// Adiciona uma nova sentença na lista de mensagem a serem eviada
// Inputs: String com a sentença a ser enviado
// Return: nada
// void LoRaNodeLib::addNewMsg(String strNewMsg) {
void LoRaNodeLib::addNewMsg(String strNewMsg, uint8_t type) {
    newSendMessages++;
    messageLoRa_t _sendMessage;
    _sendMessage.id = getLocalAddress();
    _sendMessage.type = type;
    _sendMessage.payload = strNewMsg;
    // sendMessagesList.add(strNewMsg);
    sendMessagesList.add(_sendMessage);
}

// Verifica se existe mensagens a serem enviada
// Inputs: nenhum
// Return: bool se tiver mensagens true caso contrario false
uint8_t LoRaNodeLib::isMsgForSend(void) {
    return newSendMessages;
}

// Verifica se existe mensagens a serem enviada
// Inputs: nenhum
// Return: bool se tiver mensagens true caso contrario false
float LoRaNodeLib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}

//========================================
// Private
//========================================

// Executada automaticamente todas as vezes que o modulo LoRa recebe uma nova mensagem
// Inputs: int com o tamanho do pacote que foi recebido pelo modulo LoRa
// Return: nenhum
void LoRaNodeLib::onReceive(int packetSize) {
    if (packetSize == 0) return;        // verifica o tamanho do pacote

    String _strRecv = "";

    while (LoRa.available()) {          // ler a string recebida pelo modulo LoRa
        _strRecv += (char)LoRa.read();
    }

    if (DEBUG_LORAT) {
        Serial.print("onReceive: ");
        Serial.println(_strRecv);
    }

    if (_strRecv != "") {

        strReceived = _strRecv;
        msgReceived = decodeJSON(strReceived);
        newRecvMessages = true;

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
// Inputs: nenhum
// Return: nenhum
void LoRaNodeLib::SetTxMode(void) {
    LoRa.idle();                          // set standby mode
    LoRa.disableInvertIQ();               // normal mode
}

// Converte um messageLoRa_t em uma estrutura String
// Inputs: messageLoRa_t a ser convertida
// Return: String resultante codificação da messageLoRa_t de entrada
// String LoRaNodeLib::encodeJSON(messageLoRa_t msgSend) {
//     return encodeJSON(msgSend, 99);
// }

// Converte um messageLoRa_t em uma estrutura String
// Inputs: messageLoRa_t a ser convertida
//         type tipo do dados gerado a ser convertido 
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
    
    if (DEBUG_LORAT) {
        Serial.print("strJSONRecv 2: ");
        Serial.println(strJSONRecv);
    }

    messageLoRa_t _message;

    StaticJsonDocument<200> doc;

    DeserializationError error = deserializeJson(doc, strJSONRecv);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return _message;
    }

    // if (DEBUG_LORAT) {
    //     Serial.println("Deserialization ok!");
    // }

    JsonObject root = doc.as<JsonObject>();

    String id = root["id"];
    uint8_t type = root["type"];
    String payload = root["payload"];

    _message.id = id;
    _message.type = type;
    _message.payload = payload;

    return _message;

}
