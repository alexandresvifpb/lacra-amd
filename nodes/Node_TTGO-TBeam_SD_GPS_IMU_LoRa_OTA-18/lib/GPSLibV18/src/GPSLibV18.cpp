#include "GPSLibV18.h"

GPSLib::GPSLib() {}

TinyGPSPlus gpsNEO;
boolean avaliableDatas;
gps_t lastPoint;
gps_t currentPoint;

//======================
// Funcoes Public
//======================

// 
// Inputs: nenhum
// Return: boolean
boolean GPSLib::begin(void) {
    bool result = false;

    // Inicializa a porta serial do gpsNEO
    Serial1.begin(9600, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);

    // Testa se a conexao serial com o gpsNEO foi estabelecida
    uint8_t cont = 1;
    while (cont > 0) {

        if (Serial1) {
            result = true;
            break;
        }
        
        Serial.print(".");
        cont++;
        delay(10);
    }

    if (result) {
        Serial.println("GPS sensor initialized with SUCCESS");
    } else {
        Serial.println("GPS sensor initialized with FAULT");
    }

    gpsDebugMain = GPS_DEBUG_MAIN;
    gpsDebugGetData = GPS_DEBUG_GETDATA;
    gpsTaskDelayMS = GPS_TASK_DELAY_MS;
    gpsDebugTimeSerial = GPS_DEBUG_TIME_SERIAL/gpsTaskDelayMS;
    tsSendIntervalGPS = GPS_SEND_INTERVAL * 1000;

    return result;
}

// 
// Inputs: nenhum
// Return: boolean
boolean GPSLib::read(void) {

    if (millis() - tsLastReportGPS >= tsIntervalGPS) {
        while (Serial1.available()) {

            lastPoint = currentPoint;

            gpsNEO.encode(Serial1.read());
            if (gpsNEO.location.isUpdated()) {
                parametersGPS.latitude = gpsNEO.location.lat();
                parametersGPS.longitude = gpsNEO.location.lng();
                parametersGPS.altitude = gpsNEO.altitude.meters();
                parametersGPS.speed = gpsNEO.speed.kmph();
                parametersGPS.date = String(gpsNEO.date.value());
                parametersGPS.time = String(gpsNEO.time.value());
                parametersGPS.satellites = gpsNEO.satellites.value();
                parametersGPS.hdop = gpsNEO.hdop.value();

                timeGPSNow.hours = gpsNEO.time.hour();
                timeGPSNow.minutes = gpsNEO.time.minute();
                timeGPSNow.seconds = gpsNEO.time.second();
                timeGPSNow.day = gpsNEO.date.day();
                timeGPSNow.month = gpsNEO.date.month();
                timeGPSNow.year = gpsNEO.date.year();

                avaliableDatas = true;
            }
            delay(1);

            currentPoint = parametersGPS;
        }
        tsLastReportGPS = millis();
        // requestSync();              // IMPORTANTE!!! responsavel por atualizar o relogio
        if (now() < GPS_EPOCH_TIME_2019) requestSync();
    }

    return avaliableDatas;
}

// 
// Inputs: nenhum
// Return: boolean
// boolean GPSLib::isAvaliableDatas(void) {
boolean GPSLib::avaliable(void) {
    if (avaliableDatas) {
        avaliableDatas = false;
        return true;
    } else {
        return false;
    }
}

// 
// Inputs: uint64_t
// Return: String
String GPSLib::getDataString(uint64_t unixTime) {
    // [0,1539252653,-7.225510,-35.889114,521.900024,0.170000]

    char cLatitudeTemp[10];
    sprintf(cLatitudeTemp, "%f", parametersGPS.latitude);

    char cLongitudeTemp[20];
    sprintf(cLongitudeTemp, "%f", parametersGPS.longitude);

    String result = "[";
    result += (unsigned long) unixTime;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += cLatitudeTemp;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += cLongitudeTemp;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += parametersGPS.altitude;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += parametersGPS.speed;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += parametersGPS.satellites;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += parametersGPS.hdop;
    if (gpsDebugGetData) Serial.println(result);
    result += ",";
    result += getVBat();
    if (gpsDebugGetData) Serial.println(result);
    result += "]";
    if (gpsDebugGetData) Serial.println(result);
    return result;
}

// 
// Inputs: nenhum
// Return: boolean
boolean GPSLib::requestSync(void) {
    // Set Time from GPS data string
    setTime(timeGPSNow.hours, timeGPSNow.minutes, timeGPSNow.seconds, timeGPSNow.day, timeGPSNow.month, timeGPSNow.year);
    // Calcula o tempo atual do fuso horário por valor de deslocamento
    adjustTime (UTC_offset * SECS_PER_HOUR);
    return 0;
}

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
boolean GPSLib::checkMovement(void) {
    double delta = gpsNEO.distanceBetween(lastPoint.latitude, lastPoint.longitude, currentPoint.latitude, currentPoint.longitude);

    Serial.print("Delta: ");
    Serial.println(delta);

    if (delta > GPS_POSITION_ACCURACY) return true;
    return false;
}

//======================
// Funcoes Private
//======================

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float GPSLib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
}