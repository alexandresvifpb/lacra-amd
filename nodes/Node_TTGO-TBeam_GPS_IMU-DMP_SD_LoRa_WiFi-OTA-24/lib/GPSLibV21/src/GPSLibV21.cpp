#include "GPSLibV21.h"

TinyGPSPlus gpsModule;
// HardwareSerial gpsSerial = HardwareSerial();
static const double INIT_LAT = GPS_CGE_LAT, INIT_LNG = GPS_CGE_LNG;
gps_t lastPosition, currentPosition;
gps_time_t currentDateTime;
boolean datasValid = false;
boolean datasUpdate = false;

GPSLib::GPSLib() {}

// Inicializa a porta Serial do GPS e verifica se o modulo gps esta conectado 
boolean GPSLib::begin(void) {
    boolean result = false;

    Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);

    uint8_t cont = 1;
    while (cont > 0) 
    {
        if (Serial1) {
            result = true;
            break;
        }
        
        Serial.print(".");
        cont++;
        delay(10);
    }

    (result) ? Serial.println("GPS sensor initialized with SUCCESS") : Serial.println("GPS sensor initialized with FAULT");

    return result;
}

//
void GPSLib::run(void) {
    while ( Serial1.available() ) {

/*
        Serial.println(__LINE__);

        if ( gpsModule.location.isValid() ) {
            Serial.println(__LINE__);
        } else {
            Serial.println(__LINE__);
        }

        if ( gpsModule.location.age() < 2000 ) {
            Serial.println(__LINE__);
        } else {
            Serial.println(__LINE__);
        }
*/

        ( gpsModule.location.isValid() && ( gpsModule.location.age() < 2000 ) ) ? datasValid = true : datasValid = false;
/*
        if ( !datasValid ) {

        Serial.println(__LINE__);
        
            currentPosition.age = gpsModule.location.age();
            continue;
        }

        Serial.println(__LINE__);
*/
        
        currentPosition.age = gpsModule.location.age();

        gpsModule.encode( Serial1.read() );
        if ( gpsModule.location.isUpdated() ) {

        // Serial.println(__LINE__);
        
            lastPosition = currentPosition;

            currentPosition.latitude = gpsModule.location.lat();
            currentPosition.longitude = gpsModule.location.lng();
            currentPosition.altitude = gpsModule.altitude.meters();
            currentPosition.speed = gpsModule.speed.kmph();
            currentPosition.course = gpsModule.course.deg();
            currentPosition.distanceBetweenTwoPoints = gpsModule.distanceBetween(lastPosition.latitude, lastPosition.longitude, currentPosition.latitude, currentPosition.longitude);
            currentPosition.age = gpsModule.location.age();
            currentPosition.date = String(gpsModule.date.value());
            currentPosition.time = String(gpsModule.time.value());
            currentPosition.satellites = gpsModule.satellites.value();
            currentPosition.hdop = gpsModule.hdop.value();

            currentDateTime.hours = gpsModule.time.hour();
            currentDateTime.minutes = gpsModule.time.minute();
            currentDateTime.seconds = gpsModule.time.second();
            currentDateTime.day = gpsModule.date.day();
            currentDateTime.month = gpsModule.date.month();
            currentDateTime.year = gpsModule.date.year();

            tmElements_t t;
            t.Year = currentDateTime.year - 1970;
            t.Month = currentDateTime.month;
            t.Day = currentDateTime.day;
            t.Hour = currentDateTime.hours;
            t.Minute = currentDateTime.minutes;
            t.Second = currentDateTime.seconds;
            currentDateTime.epochTime = makeTime(t) + GPS_UTC_OFFSET * 3600;

            datasUpdate = true;
        }
    }
}

//
boolean GPSLib::isLocationValid(void) {

        // Serial.println(__LINE__);
        
    return ( gpsModule.location.isValid() && ( gpsModule.location.age() < 2000 ) );
}

//
boolean GPSLib::isLocationUpdate(void) {

        // Serial.println(__LINE__);
        
    boolean ret = datasUpdate;
    datasUpdate = false;
    return ret;
}

//
boolean GPSLib::isDateTimeValid(void) {
    // return ( ( gpsModule.time.isValid() && ( gpsModule.time.age() < 2000 ) ) && ( gpsModule.date.isValid() && ( gpsModule.date.age() < 2000 ) ) );
    return ( ( currentDateTime.epochTime > GPS_MINIMUM_EPOCH_TIME ) && ( currentDateTime.epochTime < GPS_MAXIMUM_EPOCH_TIME ) );
}

//
unsigned long GPSLib::getUnixTimeNow(void) {
    return currentDateTime.epochTime;
}

// 
String GPSLib::getStringDatas(uint64_t unixTime, uint16_t bootsequence) {

    char cLatitudeTemp[10];
    sprintf(cLatitudeTemp, "%f", currentPosition.latitude);

    char cLongitudeTemp[20];
    sprintf(cLongitudeTemp, "%f", currentPosition.longitude);

    String result = String((unsigned long)unixTime);
    result += ",";
    result += bootsequence;
    result += ",";
    result += cLatitudeTemp;
    result += ",";
    result += cLongitudeTemp;
    result += ",";
    result += currentPosition.altitude;
    result += ",";
    result += currentPosition.speed;
    result += ",";
    result += currentPosition.course;
    result += ",";
    result += currentPosition.distanceBetweenTwoPoints;
    result += ",";
    result += currentPosition.age;
    result += ",";
    // result += currentPosition.date;
    // result += ",";
    // result += currentPosition.time;
    // result += ",";
    result += String((unsigned long)currentDateTime.epochTime);
    result += ",";
    result += currentPosition.satellites;
    result += ",";
    result += currentPosition.hdop;
    return result;
}