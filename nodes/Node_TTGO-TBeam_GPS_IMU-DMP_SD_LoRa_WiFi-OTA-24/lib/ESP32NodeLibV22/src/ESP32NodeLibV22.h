#ifndef ESP32NODELIBV22_H 
#define ESP32NODELIBV22_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"
#include "EEPROM.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

// #define ESP32_TIME_DELAY            (120000)            // in miles seconds
#define RESET_CONT_BOOT             (false)             // resets the bootsequence counter variable in the EEPROM

const uint64_t minUnixTime = 1577840400;             // 01/01/2020 00:00:00 
const uint64_t maxUnixTime = 1672531199;             // 01/01/2021 00:00:00

typedef struct {
    String id;
    uint64_t epochTimeGPS;
    uint64_t sequence;
    uint16_t bootSequence;
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
    float Yaw;
    float Pitch;
    float Roll;
} dataIMU_t;

typedef struct {
    String id;
    uint64_t epochTimeGPS;
    uint64_t sequence;
    uint16_t bootSequence;
    boolean inMovement;
    float latitude;
    float longitude;
    float altitude;
    float speed;
    float displacement;
    uint16_t direction;
} dataGPS_t;

typedef struct {
    String id;
    uint64_t epochTimeGPS;
    uint64_t sequence;
    uint16_t bootSequence;
    uint64_t seconds;
    String dateTime;
    dataIMU_t dataIMU;
    dataGPS_t dataGPS;
} dataALL_t;

class ESP32NodeLib
{
    public:
        ESP32NodeLib();                         // construtor

        String getIdModule(void);
        uint16_t getBootSequence(void);
        unsigned long getUnixTimeNow(void);
        void setUnixTime(unsigned long unixTime);
        boolean isValidUnixTime(uint64_t currentUnixTime);

    private:

};


#ifdef __cplusplus
}
#endif

#endif  // ESP32NODELIBV22_H