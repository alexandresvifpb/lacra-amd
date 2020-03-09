#include "GPSLibV20.h"

GPSLib::GPSLib() {}

TinyGPSPlus gpsNEO;

//======================
// Funcoes Public
//======================

// Inicializa a porta Serial do GPS e verifica se o modulo gpsNEO esta conectado 
// Inputs: void
// Return: boolean TRUE caso a comunicacao com o modulo GPS seja bem sucedida
boolean GPSLib::begin(void)
{
    bool result = false;

    // Inicializa a porta serial do gpsNEO
    Serial1.begin(9600, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);

    // Testa se a conexao serial com o gpsNEO foi estabelecida
    uint8_t cont = 1;
    while (cont > 0) 
    {

        if (Serial1) 
        {
            result = true;
            break;
        }
        
        Serial.print(".");
        cont++;
        delay(10);
    }

    if (result) 
    {
        Serial.println("GPS sensor initialized with SUCCESS");
    } else 
    {
        Serial.println("GPS sensor initialized with FAULT");
    }

    gpsTaskDelayMS = GPS_TASK_DELAY_MS;

    return result;
}

// Verifica se chegou dados na porta serial do GPS
// Inputs: void
// Return: boolean TRUE caso novo dados chegue a porta
boolean GPSLib::avaliable(void) {
    avaliableDatas = Serial1.available();
    return avaliableDatas;
}

// Recupera todos os dados enviado pelo modulo GPS e salva na variavel currentPosition e currentDateTime
// Inputs: void
// Return: void
void GPSLib::update(void)
{
    if (avaliableDatas)
    {
        while (Serial1.available()) 
        {

            // Serial.println("gps.update() #1");

            lastPosition = currentPosition;

            gpsNEO.encode(Serial1.read());

            if ( gpsNEO.location.isUpdated() ) 
            {
                currentPosition.latitude = gpsNEO.location.lat();
                currentPosition.longitude = gpsNEO.location.lng();
                currentPosition.altitude = gpsNEO.altitude.meters();
                currentPosition.speed = gpsNEO.speed.kmph();
                currentPosition.date = String(gpsNEO.date.value());
                currentPosition.time = String(gpsNEO.time.value());
                currentPosition.satellites = gpsNEO.satellites.value();
                currentPosition.hdop = gpsNEO.hdop.value();

                currentDateTime.hours = gpsNEO.time.hour();
                currentDateTime.minutes = gpsNEO.time.minute();
                currentDateTime.seconds = gpsNEO.time.second();
                currentDateTime.day = gpsNEO.date.day();
                currentDateTime.month = gpsNEO.date.month();
                currentDateTime.year = gpsNEO.date.year();

                tmElements_t t;
                t.Year = currentDateTime.year - 1970;
                t.Month = currentDateTime.month;
                t.Day = currentDateTime.day;
                t.Hour = currentDateTime.hours;
                t.Minute = currentDateTime.minutes;
                t.Second = currentDateTime.seconds;
                currentDateTime.epochTime = makeTime(t);

                avaliableDatas = true;
            }

        }
        avaliableDatas = false;
    }
}

// Calcula a distancia entre o ponto anterior e o atual
// Inputs: gps_t; gps_t com os dados da posicao anterior e atual
// Return: double valor da distancia entre os dois pontos
double GPSLib::calculateDelta(gps_t position1, gps_t position2)
{
    return gpsNEO.distanceBetween(position1.latitude, position1.longitude, position2.latitude, position2.longitude);
}

// Gera uma string com todos os dados recuperado do modulo GPS
// Inputs: uint64_t epoch time do momento que os dados do modulo GPS foram recuperado
// Return: String com todos os dados salvo na currentPosition
String GPSLib::getCurretPosition(uint64_t unixTime) 
{
    // [0,1539252653,-7.225510,-35.889114,521.900024,0.170000]

    char cLatitudeTemp[10];
    sprintf(cLatitudeTemp, "%f", currentPosition.latitude);

    char cLongitudeTemp[20];
    sprintf(cLongitudeTemp, "%f", currentPosition.longitude);

    String result = "[";
    result += (unsigned long) unixTime;
    result += ",";
    result += cLatitudeTemp;
    result += ",";
    result += cLongitudeTemp;
    result += ",";
    result += currentPosition.altitude;
    result += ",";
    result += currentPosition.speed;
    result += ",";
    result += currentPosition.satellites;
    result += ",";
    result += currentPosition.hdop;
    result += ",";
    result += "]";

    return result;
}

// Verifica se o intervalo entre um novo registro foi atingido
// Inputs: void
// Return: boolean TRUE caso o intervalo seja esgotado
boolean GPSLib::checkTimeInterval(void) 
{
    if (millis() - tsLastReportGPS >= tsIntervalGPS)
    {
        tsLastReportGPS = millis();
        return true;
    }

    return false;
}

// Sincroniza o time do ESP32 com o relogio do GPS
// Inputs: void 
// Return: boolean TRUE (sempre) precisa criar uma regra caso o ajuste nao seja bem sucedido
boolean GPSLib::requestSync(void) 
{
    // Set Time from GPS data string
    setTime(currentDateTime.hours, currentDateTime.minutes, currentDateTime.seconds, currentDateTime.day, currentDateTime.month, currentDateTime.year);
    // Calcula o tempo atual do fuso horário por valor de deslocamento
    adjustTime (UTC_offset * SECS_PER_HOUR);

    if ( currentDateTime.year >= 2020 )
    {
        return true;
    } 
    else
    {
        return false;
    }
    
}

// Gera uma string com todos os dados recuperado do modulo GPS
// Inputs: uint64_t epoch time do momento que os dados do modulo GPS foram recuperado
// Return: String com todos os dados salvo na currentPosition
uint64_t GPSLib::getEpochTimeNow(void)
{
    return currentDateTime.epochTime;
}

// Gera uma string com todos os dados recuperado do modulo GPS
// Inputs: uint64_t epoch time do momento que os dados do modulo GPS foram recuperado
// Return: String com todos os dados salvo na currentPosition
String GPSLib::getDateTimeNow(void)
{
    return String(currentDateTime.day) + '/' + String(currentDateTime.month) + '/' + String(currentDateTime.year) + '\t' + String(currentDateTime.hours) + ':' + String(currentDateTime.minutes) + ':' + String(currentDateTime.seconds);
}