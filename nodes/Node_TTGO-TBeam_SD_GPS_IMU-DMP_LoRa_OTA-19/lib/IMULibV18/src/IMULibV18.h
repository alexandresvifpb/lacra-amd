#include <MPU9250.h>

#ifndef IMULIBV18_H
#define IMULIBV18_H

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define IMU_TYPE                        (3)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_ACCEL_TYPE                  (4)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_GYROM_TYPE                  (5)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_MAGNE_TYPE                  (6)         // define como sendo modulo do tipo 2 (MPU9255)

// #define MPU_ID                          (12345)     // define um ID para o modulo
#define MPU_MPU9250_ADDRESS             (0x68)  // Device address when ADO = 0
#define MPU_PIN_SDA                     (21)            //(36)     //(GPIO_NUM_10)
#define MPU_PIN_SCL                     (22)            //(39)     //(GPIO_NUM_9)
#define PIN_VBAT                        (35)  // GPIO35 - pino para leitura da Vpp da bateria

#define ACGYMG_REPORTING_PERIOD_MS      (20)
#define DATA_ACCEL_DIF                  (0.5)

#define IMU_TASK_DELAY_MS               (50)

#define MADGWICK_FILTER                 (0)         // MadgwickQuaternionUpdate()
#define MAHONY_FILTER                   (1)         // MahonyQuaternionUpdate()

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

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

class IMULib
{
    public:

        IMULib();

        boolean begin(void);
        void read(void);
        boolean avaliable(void);
        String getStrDataIMU(uint64_t unixTime);
        int readFifo(void);
        String getStrFifoDataIMU(uint64_t unixTime);

        boolean calibrateAccel(void);
        boolean calibrateGyro(void);
        boolean calibrateMag(void);

        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

        void updateQuartenion(uint8_t filter);
        float getPitch(void);
        float getRoll(void);
        float getYaw(void);

        IMU_t valuesIMU;
        boolean newValuesIMU;
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

#endif /* IMULIBV18_H */