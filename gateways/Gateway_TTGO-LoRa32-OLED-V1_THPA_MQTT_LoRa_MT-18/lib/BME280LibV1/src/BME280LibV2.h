#ifndef BME280LIBV2_H
#define BME280LIBV2_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define THPA_TYPE                   (8)                 // define como sendo modulo do tipo 2 (MPU9255)
#define THPA_PIN_SDA                (GPIO_NUM_21)       //(36)     //(GPIO_NUM_10)
#define THPA_PIN_SCL                (GPIO_NUM_22)       //(39)     //(GPIO_NUM_9)
#define THPA_PIN_VBAT               (35)                // GPIO35 - pino para leitura da Vpp da bateria

#define THPA_SEALEVELPRESSURE_HPA   (1013.25)
#define THPA_ADDRESS                (0x76)
#define THPA_INTERVAL               (10)

#define THPA_DEBUG_MAIN             (false)
#define THPA_TASK_DELAY_MS          (5000)
#define THPA_DEBUG_MESSAGE          ("THPA_TASK: ")
#define THPA_DEBUG_TIME_SERIAL      (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib


typedef struct 
{
    float temperature;
    float humidity;
    float pressure;
    float altitude;
} THPA_t;

class BME280Lib
{
    public:

        BME280Lib();

        boolean begin(void);
        boolean read(void);
        boolean avaliable(void);
        String getStrDataTHPA(uint64_t unixTime, String addressID);

        boolean thpaDebugMain;
        uint16_t thpaTaskDelayMS;
        uint16_t thpaDebugTimeSerial;

    private:
        float getVBat(void);

        uint8_t sensorId;
        uint64_t tsLastReportTHPA = millis();
        uint16_t tsIntervalTHPA = (THPA_INTERVAL - 1) * 1000;
        const uint8_t vbatPin = THPA_PIN_VBAT;
        const uint16_t sealevelpressure = THPA_SEALEVELPRESSURE_HPA;
        THPA_t valuesTHPA;
        bool newValuesTHPA;

};

#ifdef __cplusplus
}
#endif

#endif /* BME280LIBV2_H */