#include <SparkFunMPU9250-DMP.h>

#ifndef IMULIBV20_H
#define IMULIBV20_H

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define IMU_TYPE                        (3)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_ACCEL_TYPE                  (4)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_GYROM_TYPE                  (5)         // define como sendo modulo do tipo 2 (MPU9255)
#define MPU_MAGNE_TYPE                  (6)         // define como sendo modulo do tipo 2 (MPU9255)

// #define MPU_ID                       (12345)     // define um ID para o modulo
#define MPU_MPU9250_ADDRESS             (0x68)      // Device address when ADO = 0
#define MPU_PIN_SDA                     (21)        //(36)     //(GPIO_NUM_10)
#define MPU_PIN_SCL                     (22)        //(39)     //(GPIO_NUM_9)
#define PIN_VBAT                        (35)        // GPIO35 - pino para leitura da Vpp da bateria

#define ACGYMG_REPORTING_PERIOD_MS      (20)
#define DATA_ACCEL_DIF                  (0.2)
#define DATA_Yaw_DIF                    (0.2)
#define DATA_PITCH_DIF                  (0.2)
#define DATA_ROLL_DIF                   (0.2)

#define IMU_TASK_DELAY_MS               (10)
#define IMU_SEND_LOOP_DELAY_S           (120)

#define MADGWICK_FILTER                 (0)         // MadgwickQuaternionUpdate()
#define MAHONY_FILTER                   (1)         // MahonyQuaternionUpdate()

typedef struct 
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
} IMU_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

typedef struct {
    float Yaw;
    float Pitch;
    float Roll;
} EulerAngles_t;

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

enum DataStringIMU {
  DSI_ACGYMGT = 0,  // Accel, Gyro, Mag, Temperature
  DSI_QTEA          // Quaternion, EulerAngles
};

class IMULib
{
    public:

        IMULib();

        boolean begin(void);
        boolean avaliable(void);
        boolean updateDatas(void);

        IMU_t getIMU(void);
        Quaternion_t getQuaternion(void);
        EulerAngles_t getEulerAngles(void);
        EulerAngles_t toEulerianAngle(Quaternion_t quaternion);

        String getStrDatas(uint64_t unixTime, IMU_t datasIMU);
        String getStrDatas(uint64_t unixTime, Quaternion_t datasQuaternion);
        String getStrDatas(uint64_t unixTime, EulerAngles_t datasEulerAngles);
        String getStrDatas(uint64_t unixTime, IMU_t datasIMU, Quaternion_t datasQuaternion);
        String getStrDatas(uint64_t unixTime, IMU_t datasIMU, EulerAngles_t datasEulerAngles);
        String getStrDatas(uint64_t unixTime, Quaternion_t datasQuaternion, EulerAngles_t datasEulerAngles);
        String getStrDatas(uint64_t unixTime, IMU_t datasIMU, Quaternion_t datasQuaternion, EulerAngles_t datasEulerAngles);

        uint32_t imuTaskDelayMS;
        uint32_t imuSendLoopDelayS;

    private:

        String getStrIMU(IMU_t datasIMU);
        String getStrQuaternion(Quaternion_t datasQuaternion);
        String getStrEulerAngles(EulerAngles_t datasEulerAngles);
};

#ifdef __cplusplus
}
#endif

#endif /* IMULIBV20_H */