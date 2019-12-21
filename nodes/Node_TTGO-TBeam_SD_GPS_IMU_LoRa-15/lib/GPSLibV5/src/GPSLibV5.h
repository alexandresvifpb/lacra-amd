#ifndef GPSLIBV5_H
#define GPSLIBV5_H

#include "ESP32LibV2.h"
#include <TinyGPS++.h>

#ifdef __cplusplus
extern "C" {
#endif

// #define BUF_SIZE                (1024)
#define GPS_PIN_RX              (12)        //(GPIO_NUM_12)             //(39)     //(GPIO_NUM_9)
#define GPS_PIN_TX              (15)        //(GPIO_NUM_15)            //(36)     //(GPIO_NUM_10)
#define GPS_PIN_VBAT            (35)        // GPIO35 - pino para leitura da Vpp da bateria

#define GPS_TYPE                (1)         // Codigo dos dados sobre GPS
#define GPS_UTC_OFFSET          (-3)
#define GPS_INTERVAL            (5)
#define GPS_EPOCH_TIME_2019     (1546300800)        // 00:00:00 01/01/2019

#define GPS_DEBUG_MAIN          (true)
#define GPS_DEBUG_GETDATA       (false)
#define GPS_DEBUG_TIME_SERIAL   (120000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define GPS_DEBUG_MESSAGE       ("GPS_TASK: ")
#define GPS_TASK_DELAY_MS       (120000)

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

        boolean begin(void);
        boolean read(void);
        String getDataString(uint64_t unixTime);
        boolean avaliable(void);
        boolean requestSync(void);

        boolean gpsDebugMain;
        boolean gpsDebugGetData;
        uint32_t gpsDebugTimeSerial;
        uint32_t gpsTaskDelayMS;
        gps_t parametersGPS;

    private:
        float getVBat(void);

        gps_time_t timeGPSNow;
        const int UTC_offset = GPS_UTC_OFFSET;
        time_t unixtimeGPS;
        uint32_t tsLastReportGPS = 0;
        uint16_t tsIntervalGPS = (GPS_INTERVAL - 1) * 1000;
        const uint8_t vbatPin = GPS_PIN_VBAT;

};

#ifdef __cplusplus
}
#endif

#endif // GPSLIBV5_H