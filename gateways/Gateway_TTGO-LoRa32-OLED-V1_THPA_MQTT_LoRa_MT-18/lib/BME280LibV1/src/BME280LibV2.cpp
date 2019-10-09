#include "BME280LibV2.h"

Adafruit_BME280 bme;
boolean status = false;
boolean newReadDatas = false;

BME280Lib::BME280Lib() {}

boolean BME280Lib::begin(void) {

    status = bme.begin(THPA_ADDRESS);

    if (status) {
        // Serial.println("BME280 initialization successful");

        thpaDebugMain = THPA_DEBUG_MAIN;
        thpaTaskDelayMS = THPA_TASK_DELAY_MS;
        thpaDebugTimeSerial = THPA_DEBUG_TIME_SERIAL/thpaTaskDelayMS;

        return true;
    }

    return false;
}

boolean BME280Lib::read(void) {
    valuesTHPA.temperature = bme.readTemperature();
    valuesTHPA.humidity = bme.readHumidity();
    valuesTHPA.pressure = bme.readPressure() / 100.0F;
    valuesTHPA.altitude = bme.readAltitude(sealevelpressure);
    newReadDatas = true;
    return true;
}

boolean BME280Lib::avaliable(void) {
    bool result = newReadDatas;
    newReadDatas = false;
    return result;
}

String BME280Lib::getStrDataTHPA(uint64_t unixTime, String addressID) {
    String _strJSON;

    _strJSON = "{\"id\":\"";
    _strJSON.concat(addressID);
    _strJSON.concat("\",");
    _strJSON.concat("\"type\":");
    _strJSON.concat(THPA_TYPE);
    _strJSON.concat(",");
    _strJSON.concat("\"payload\":");
    _strJSON.concat("[");
    _strJSON.concat((unsigned long)unixTime);
    _strJSON.concat(",");
    _strJSON.concat(String(valuesTHPA.temperature,2));
    _strJSON.concat(",");
    _strJSON.concat(String(valuesTHPA.humidity,2));
    _strJSON.concat(",");
    _strJSON.concat(String(valuesTHPA.pressure,2));
    _strJSON.concat(",");
    _strJSON.concat(String(valuesTHPA.altitude,2));
    _strJSON.concat("]}");

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