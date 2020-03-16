#ifndef GPSLIBV21_H
#define GPSLIBV21_H

#include <TinyGPS++.h>
#include <Time.h>
#include "TimeLib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_PIN_RX              (12)                //(GPIO_NUM_12)             //(39)     //(GPIO_NUM_9)
#define GPS_PIN_TX              (15)                //(GPIO_NUM_15)            //(36)     //(GPIO_NUM_10)
#define GPS_BAUD                (9600)

#define GPS_CGE_LAT             (-7.226357)
#define GPS_CGE_LNG             (-35.88868)

#define GPS_TYPE                (3)
#define GPS_UTC_OFFSET          (-3)
#define GPS_MINIMUM_EPOCH_TIME  (1577836800)        // 00:00:00 01/01/2020
#define GPS_MAXIMUM_EPOCH_TIME  (1647371693)        // 00:00:00 01/01/2020

#define GPS_TASK_DELAY_MS       (500)
#define GPS_SEND_INTERVAL       (120)               // Intervalo (em segundos) para envio dos dados GPS para o Gateway

#define GPS_POSITION_ACCURACY   (2.0)

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double course;
    double distanceBetweenTwoPoints;
    uint32_t age;
    String date;
    String time;
    unsigned long epochTime;
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

class GPSLib {
    public:

        GPSLib();     // construtor

        boolean begin(void);
        void run(void);
        boolean isLocationValid(void);
        boolean isLocationUpdate(void);
        boolean isDateTimeValid(void);
        unsigned long getUnixTimeNow(void);
        String getStringDatas(uint64_t unixTime, uint16_t bootsequence);
        uint32_t tsGPSTaskDelayMS = GPS_TASK_DELAY_MS;

        // boolean avaliable(void);
        // void update(void);
        // double calculateDelta(gps_t position1, gps_t position2);
        // String getCurretPosition(uint64_t unixTime);
        // boolean checkTimeInterval(void);
        // boolean requestSync(void);
        // uint64_t getEpochTimeNow(void);
        // String getDateTimeNow(void);

        // gps_t lastPosition, currentPosition;

    private:

        // boolean avaliableDatas;
        // gps_time_t currentDateTime;
        // const int UTC_offset = GPS_UTC_OFFSET;
        // uint32_t tsLastReportGPS = 0;
        // uint32_t tsIntervalGPS = (GPS_SEND_INTERVAL) * 1000;             // em segundos


        // gps_time_t timeGPSNow;
        // const int UTC_offset = GPS_UTC_OFFSET;
        // time_t unixtimeGPS;
        // const uint8_t vbatPin = GPS_PIN_VBAT;


};

#ifdef __cplusplus
}
#endif

#endif // GPSLIBV21_H