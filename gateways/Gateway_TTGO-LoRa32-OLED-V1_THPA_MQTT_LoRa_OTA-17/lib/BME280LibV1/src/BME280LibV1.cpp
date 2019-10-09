#include "BME280LibV1.h"

Adafruit_BME280 bme;
boolean status = false;
boolean newReadDatas = false;

BME280Lib::BME280Lib() {}

bool BME280Lib::begin(void) {

    status = bme.begin(THPA_ADDRESS);

    if (status) {
        Serial.println("BME280 initialization successful");
        return true;
    }

    return false;
}

void BME280Lib::run(void) {
    if (millis() - tsLastReportTHPA >= tsIntervalTHPA) {
        newReadDatas = true;
        tsLastReportTHPA = millis();
    }
}

bool BME280Lib::avaliable(void) {
    bool result = newReadDatas;
    newReadDatas = false;
    return result;
}

String BME280Lib::getStrDataTHPA(uint64_t unixTime) {
    String result = "[";
    // result += THPA_TYPE;
    // result += ",";
    result += (unsigned long)unixTime;
    result += ",";
    result += String(bme.readTemperature(),2);
    result += ",";
    result += String(bme.readHumidity(),2);
    result += ",";
    result += String(bme.readPressure() / 100.0F,2);
    result += ",";
    result += String(bme.readAltitude(sealevelpressure),2);
    result += ",";
    result += getVBat();
    result += "]";
    return result;
}

String BME280Lib::getStrDataTHPA(uint64_t unixTime, String addressID) {
    /*
    String result = "[";
    // result += THPA_TYPE;
    // result += ",";
    result += (unsigned long)unixTime;
    result += ",";
    result += String(bme.readTemperature(),2);
    result += ",";
    result += String(bme.readHumidity(),2);
    result += ",";
    result += String(bme.readPressure() / 100.0F,2);
    result += ",";
    result += String(bme.readAltitude(sealevelpressure),2);
    result += ",";
    result += getVBat();
    result += "]";
    return result;
    */
    String _strJSON;

    _strJSON = "{\"id\":\"";
    _strJSON.concat(addressID);
    _strJSON.concat("\",");
    _strJSON.concat("\"type\":");
    _strJSON.concat(THPA_TYPE);
    _strJSON.concat(",");
    _strJSON.concat("\"payload\":\"");
    _strJSON.concat("[");
    _strJSON.concat((unsigned long)unixTime);
    _strJSON.concat(",");
    _strJSON.concat(String(bme.readTemperature(),2));
    _strJSON.concat(",");
    _strJSON.concat(String(bme.readHumidity(),2));
    _strJSON.concat(",");
    _strJSON.concat(String(bme.readPressure() / 100.0F,2));
    _strJSON.concat(",");
    _strJSON.concat(String(bme.readAltitude(sealevelpressure),2));
    _strJSON.concat("]\"}");

    return _strJSON;
}

//======================
// Private
//======================

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float BME280Lib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}