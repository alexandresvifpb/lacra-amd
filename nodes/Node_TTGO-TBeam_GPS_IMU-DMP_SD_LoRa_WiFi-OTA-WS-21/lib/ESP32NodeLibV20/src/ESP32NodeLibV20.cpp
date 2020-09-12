#include "ESP32NodeLibV20.h"

//======================
// Funcoes Public
//======================

// Funcao construtor da classe ESP32Lib
ESP32NodeLib::ESP32NodeLib() {}

// Utiliza o MAC do modulo ESP32 para gerar um ID do modulo
// Input: nenhum
// Return: String - formada com parte do MAC (3 bytes) do modulo
String ESP32NodeLib::getIdModule(void) {
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

// Seta a data e hora no RTC interno do ESP32 
// Inputs: unsigned long - valor unix time para a data hora atual
// Return: void
unsigned long ESP32NodeLib::getUnixTimeNow(void)
{
    return now();
}

// Seta a data e hora no RTC interno do ESP32 
// Inputs: unsigned long - valor unix time para a data hora atual
// Return: void
void ESP32NodeLib::setUnixTime(unsigned long unixTime)
{
    setTime( unixTime );
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

// 
// Inputs: Config_t
// Return: String
String ESP32NodeLib::encoderStrAllDatas(allDatas_t datas) {
    String rest;
    char cLatitudeTemp[10];
    sprintf(cLatitudeTemp, "%f", datas.latitude);

    char cLongitudeTemp[20];
    sprintf(cLongitudeTemp, "%f", datas.longitude);

    rest = "[";
    rest += datas.id;
    rest += ";";
    rest += datas.dateTime;
    rest += ";";
    rest += (unsigned long)datas.epochTimeIMU;
    rest += ";";
    rest += datas.accelX;
    rest += ";";
    rest += datas.accelY;
    rest += ";";
    rest += datas.accelZ;
    rest += ";";
    rest += datas.gyroX;
    rest += ";";
    rest += datas.gyroY;
    rest += ";";
    rest += datas.gyroZ;
    rest += ";";
    rest += datas.magX;
    rest += ";";
    rest += datas.magY;
    rest += ";";
    rest += datas.magZ;
    rest += ";";
    rest += datas.q0;
    rest += ";";
    rest += datas.q1;
    rest += ";";
    rest += datas.q2;
    rest += ";";
    rest += datas.q3;
    rest += ";";
    rest += datas.Yam;
    rest += ";";
    rest += datas.Pitch;
    rest += ";";
    rest += datas.Roll;
    rest += ";";
    rest += (unsigned long)datas.epochTimeGPS;
    rest += ";";
    rest += cLatitudeTemp;
    rest += ";";
    rest += cLongitudeTemp;
    rest += ";";
    rest += datas.altitude;
    rest += ";";
    rest += datas.speed;
    rest += "]";

    return rest;
}

//
// Inputs:  String id - endereco do dispositivo
// Return: String retorna uma string com os dados codificado em JSON
commands_t ESP32NodeLib::getCommand(String sentence) {

    commands_t _result;
    _result.type = 0;
    _result.payload = "";

    StaticJsonDocument<200> _doc;

    DeserializationError _error = deserializeJson(_doc, sentence);

    if (_error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(_error.c_str());
        return _result;
    }

    JsonObject _root = _doc.as<JsonObject>();

    uint8_t _type = _root["type"];
    String _payload = _root["payload"];

    _result.type = _type;
    _result.payload = _payload;

    return _result;

}