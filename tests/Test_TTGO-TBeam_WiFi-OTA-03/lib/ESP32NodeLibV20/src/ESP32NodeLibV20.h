#ifndef ESP32NODELIBV20_H 
#define ESP32NODELIBV20_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_TIME_DELAY            (120000)            // em mili segundos

typedef struct {
    String id;
    String dateTime;
    uint64_t epochTimeIMU;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
    float q0;
    float q1;
    float q2;
    float q3;
    float Yam;
    float Pitch;
    float Roll;
    uint64_t epochTimeGPS;
    float latitude;
    float longitude;
    float altitude;
    float speed;
} allDatas_t;

class ESP32NodeLib
{
    public:
        ESP32NodeLib();                         // construtor

        String getIdModule(void);

        unsigned long getUnixTimeNow(void);
        void setUnixTime(unsigned long unixTime);

        String encoderJSon(String id, uint8_t type, String payload);
        String encoderStrAllDatas(allDatas_t datas);

        uint64_t timeTaskDelayMS = ESP32_TIME_DELAY;

    private:
        timeval tv;                             //Cria a estrutura temporaria para funcao abaixo.


};


#ifdef __cplusplus
}
#endif

#endif  // ESP32NODELIBV20_H