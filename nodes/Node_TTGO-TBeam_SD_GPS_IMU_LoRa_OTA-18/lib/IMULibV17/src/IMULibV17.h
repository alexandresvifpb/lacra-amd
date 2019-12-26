// #include <Arduino.h>
#include <MPU9250.h>

#ifndef IMULIBV17_H
#define IMULIBV17_H

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define IMU_TYPE                        (3)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_ACCEL_TYPE                  (4)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_GYROM_TYPE                  (5)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_MAGNE_TYPE                  (6)         // define como sendo modulo do tipo 2 (MPU9255)

// #define MPU_ID                          (12345)     // define um ID para o modulo
#define PIN_MPU_SDA                     (21)            //(36)     //(GPIO_NUM_10)
#define PIN_MPU_SCL                     (22)            //(39)     //(GPIO_NUM_9)
#define PIN_VBAT                        (35)  // GPIO35 - pino para leitura da Vpp da bateria

#define ACGYMG_REPORTING_PERIOD_MS      (20)
#define DATA_ACCEL_DIF                  (0.5)

#define IMU_TASK_DELAY_MS               (50)

typedef struct 
{
    float accelX;
    float accelY;
    float accelZ;
    float accelSqrt;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
    float magDirection;
    float pressure;
    float temperature_c;
} IMU_t;

class IMULib
{
    public:

        IMULib();

        bool begin(void);
        // void run(void);
        void read(void);
        bool avaliable(void);
        String getStrDataIMU(uint64_t unixTime);

        IMU_t valuesIMU;
        bool newValuesIMU;
        uint16_t IMUTaskDelayMS;
        uint64_t numberOfRegisterIMU;
        boolean sendDataForLora = false;

    private:
        float getVBat(void);

        uint8_t     sensorId;
        uint64_t    tsLastReport = millis();
        float       dataAccelDif = DATA_ACCEL_DIF;
        const uint8_t vbatPin = PIN_VBAT;

        // float lastAccelX;
        // float lastAccelY;
        // float lastAccelZ;
        // float lastAccelSqrt;
        // float lastGyroX;
        // float lastGyroY;
        // float lastGyroZ;
        uint16_t lastMagX;
        uint16_t lastMagY;
        uint16_t lastMagZ;

};

#ifdef __cplusplus
}
#endif

#endif /* IMULIBV17_H */