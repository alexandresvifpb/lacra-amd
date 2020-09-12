#include "GPSNeo6MLibV3.h"

GPSNeo6MLib::GPSNeo6MLib() {}

TinyGPSPlus gpsNEO;
boolean avaliableDatas;

bool GPSNeo6MLib::begin(void) {
    bool result = false;

    // Inicializa a porta serial do gpsNEO
    Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

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

    // Serial.println("GPS_#1.4");

    if (result) {
        Serial.println("GPS sensor initialized with SUCCESS");
    } else {
        Serial.println("GPS sensor initialized with FAULT");
    }

    return result;
}

void GPSNeo6MLib::run(void) {

    if (millis() - tsLastReportGPS >= tsIntervalGPS) {
        while (Serial1.available()) {
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
        }
        tsLastReportGPS = millis();
    }

}

bool GPSNeo6MLib::isAvaliableDatas(void) {
    if (avaliableDatas) {
        avaliableDatas = false;
        return true;
    } else {
        return false;
    }
}

String GPSNeo6MLib::getStringDataGPS(uint64_t unixtime) {
    // [0,1539252653,-7.225510,-35.889114,521.900024,0.170000]

    char cLatitudeTemp[10];
    sprintf(cLatitudeTemp, "%f", parametersGPS.latitude);

    char cLongitudeTemp[20];
    sprintf(cLongitudeTemp, "%f", parametersGPS.longitude);

    String result = "[";
    // result += GPS_TYPE;
    // result += ",";
    result += (unsigned long) unixtime;
    result += ",";

    // Serial.println("getStringParameterGPS_#1");

    result += cLatitudeTemp;
    // result += floatToString(parametersGPS.latitude);
    result += ",";

    // Serial.println("getStringParameterGPS_#2");

    result += cLongitudeTemp;
    // result += floatToString(parametersGPS.longitude);

    // Serial.println("getStringParameterGPS_#3");

    result += ",";
    result += parametersGPS.altitude;
    result += ",";
    result += parametersGPS.speed;
    result += ",";
    result += parametersGPS.satellites;
    result += ",";
    result += parametersGPS.hdop;
    result += ",";
    result += getVBat();
    result += "]";

    // Serial.println("getStringParameterGPS_#4");

    return result;
}

time_t GPSNeo6MLib::getUnixTime(void) {
    return now();
}

bool GPSNeo6MLib::requestSync(void) {
    // Set Time from GPS data string
    setTime(timeGPSNow.hours, timeGPSNow.minutes, timeGPSNow.seconds, timeGPSNow.day, timeGPSNow.month, timeGPSNow.year);
    // Calcula o tempo atual do fuso horário por valor de deslocamento
    adjustTime (UTC_offset * SECS_PER_HOUR);
    return 0;
}

//======================
// Private
//======================

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float GPSNeo6MLib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}
