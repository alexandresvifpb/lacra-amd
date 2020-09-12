#ifndef GPSLIBV20_H
#define GPSLIBV20_H

#include <TinyGPS++.h>
#include <Time.h>
#include "TimeLib.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define BUF_SIZE                (1024)
#define GPS_PIN_RX              (12)                //(GPIO_NUM_12)             //(39)     //(GPIO_NUM_9)
#define GPS_PIN_TX              (15)                //(GPIO_NUM_15)            //(36)     //(GPIO_NUM_10)

// #define GPS_TYPE                (1)                 // Codigo dos dados sobre GPS
#define GPS_TYPE                3
#define GPS_UTC_OFFSET          (-3)
#define GPS_EPOCH_TIME_2019     (1577836800)        // 00:00:00 01/01/2020

#define GPS_DEBUG_MAIN          (true)
#define GPS_DEBUG_GETDATA       (false)

#define GPS_DEBUG_MESSAGE       ("GPS_TASK: ")
#define GPS_TASK_DELAY_MS       (1000)
#define GPS_SEND_INTERVAL       (120)               // Intervalo (em segundos) para envio dos dados GPS para o Gateway

// #define GPS_POSITION_ACCURACY   (2)                 // A (Horizontal position accuracy) para o modulo NEO 6M eh de 2,5 metros
#define GPS_POSITION_ACCURACY   2.0

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    String date;
    String time;
    uint32_t satellites;
    int32_t hdop;
} gps_t;

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint64_t epochTime;
} gps_time_t;

class GPSLib
{
    public:

        GPSLib();     // construtor

        boolean begin(void);
        boolean avaliable(void);
        void update(void);
        double calculateDelta(gps_t position1, gps_t position2);
        String getCurretPosition(uint64_t unixTime);
        boolean checkTimeInterval(void);
        boolean requestSync(void);
        uint64_t getEpochTimeNow(void);
        String getDateTimeNow(void);

        uint32_t gpsTaskDelayMS;
        gps_t lastPosition, currentPosition;

    private:
        boolean avaliableDatas;
        gps_time_t currentDateTime;
        const int UTC_offset = GPS_UTC_OFFSET;
        uint32_t tsLastReportGPS = 0;
        uint32_t tsIntervalGPS = (GPS_SEND_INTERVAL) * 1000;             // em segundos


        // gps_time_t timeGPSNow;
        // const int UTC_offset = GPS_UTC_OFFSET;
        // time_t unixtimeGPS;
        // const uint8_t vbatPin = GPS_PIN_VBAT;


};

#ifdef __cplusplus
}
#endif

#endif // GPSLIBV20_H