#ifndef GPSLIBV4_H
#define GPSLIBV4_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include "TimeLib.h"

#ifdef __cplusplus
extern "C" {
#endif

// #define BUF_SIZE            (1024)
#define PIN_GPS_RX          (12)        //(GPIO_NUM_12)             //(39)     //(GPIO_NUM_9)
#define PIN_GPS_TX          (15)        //(GPIO_NUM_15)            //(36)     //(GPIO_NUM_10)
#define PIN_GPS_VBAT        (35)        // GPIO35 - pino para leitura da Vpp da bateria

#define GPS_TYPE            (1)         // Codigo dos dados sobre GPS
// #define MAX_FAULT_COUNTER   (60)
#define UTC_OFFSET          (-3)
// #define TIME_ADJUSTMENT     (3600)                      // adjustment

#define GPS_INTERVAL        (5)

typedef struct {
    float latitude;
    float longitude;
    float speed;
    float altitude;
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
} gps_time_t;

class GPSLib
{
    public:

        GPSLib();     // construtor

        bool begin(void);
        void run(void);
        bool isAvaliableDatas(void);
        String getStringDataGPS(uint64_t unixtime);
        uint64_t getEpochTime(void);
        bool requestSync(void);
        gps_t parametersGPS;

    private:
        float getVBat(void);

        gps_time_t timeGPSNow;
        const int UTC_offset = UTC_OFFSET;
        time_t unixtimeGPS;
        uint32_t tsLastReportGPS = 0;
        uint16_t tsIntervalGPS = (GPS_INTERVAL - 1) * 1000;
        const uint8_t vbatPin = PIN_GPS_VBAT;

};

#ifdef __cplusplus
}
#endif

#endif // GPSLIBV4_H