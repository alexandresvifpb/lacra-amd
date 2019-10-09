#ifndef BME280LIBV1_H
#define BME280LIBV1_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define THPA_TYPE               (8)         // define como sendo modulo do tipo 2 (MPU9255)
// #define MPU_ID                          (12345)     // define um ID para o modulo
#define THPA_PIN_SDA            (GPIO_NUM_21)            //(36)     //(GPIO_NUM_10)
#define THPA_PIN_SCL            (GPIO_NUM_22)            //(39)     //(GPIO_NUM_9)
#define PIN_VBAT                (35)  // GPIO35 - pino para leitura da Vpp da bateria

#define SEALEVELPRESSURE_HPA    (1013.25)
#define THPA_ADDRESS            (0x76)
#define THPA_INTERVAL           (10)


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

        bool begin(void);
        void run(void);
        bool avaliable(void);
        String getStrDataTHPA(uint64_t unixTime);
        String getStrDataTHPA(uint64_t unixTime, String addressID);

        THPA_t valuesIMU;
        bool newValuesIMU;

    private:
        float getVBat(void);

        uint8_t sensorId;
        uint64_t tsLastReportTHPA = millis();
        uint16_t tsIntervalTHPA = (THPA_INTERVAL - 1) * 1000;
        const uint8_t vbatPin = PIN_VBAT;
        const uint16_t sealevelpressure = SEALEVELPRESSURE_HPA;

};

#ifdef __cplusplus
}
#endif

#endif /* BME280LIBV1_H */