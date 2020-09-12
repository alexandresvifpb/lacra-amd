#ifndef MAX30100_SENSOR_H
#define MAX30100_SENSOR_H

#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     (2000)
#define DEBUG                   (true)          // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define HRSPO2_TYPE             (6)             // define como sendo modulo do tipo 6 (MAX30100)
#define PIN_VBAT                (35)            // GPIO35 - pino para leitura da Vpp da bateria

typedef struct {
    uint16_t HR;
    uint8_t SpO2;
    float Temperature;
} SensorHRSpO2_t;

class HRSpO2 {
public:
    HRSpO2();

    bool begin(void);
    void run(void);
    bool avaliable(void);
    String getStrParamHRSpO2(uint64_t unixTime);

    SensorHRSpO2_t paramHRSpO2;
    bool newDatas = false;

private:
    float getVBat(void);
    uint32_t tsLastReport = 0;
    const uint8_t vbatPin = PIN_VBAT;

    // void (*onBeatDetected)();
};

#endif  //MAX30100_SENSOR_H
