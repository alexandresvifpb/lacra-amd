#pragma once
#ifndef MPU9250LIBV1_H
#define MPU9250LIBV1_H

#include "Arduino.h"
#include "Wire.h"     // biblioteca I2C
#include "SPI.h"      // biblioteca SPI
#include "MPU9250Lib_RegisterMap.h"

#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint16_t magX;
    uint16_t magY;
    uint16_t magZ;
    float temperature_c;
} MPU_t;

typedef struct {
    float q0;
    float qx;
    float qy;
    float qz;
} Quaternion_t;

typedef struct {
    float Yam;
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

class MPU9250Lib {
    public:
        MPU9250Lib();

        boolean begin(void);

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
        uint8_t readByte(uint8_t address, uint8_t subAddress);
        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

        void readAccelData(int16_t * destination);
        void readGyroData(int16_t * destination);
        void readMagData(int16_t * destination);
        int16_t readTempData(void);

        void initMPU9250(void);
        void initAK8963(float * destination);

        void calibrateMPU9250(float * dest1, float * dest2);

        void selfTestMPU9250(float * destination);

        void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
        void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

        void getAres(void);
        void getGres(void);
        void getMres(void);

        MPU_t getRegisters(void);
        Quaternion_t getQuaternion(void);
        float getDeltat(void);
        void setDeltat(float value);
        float getPitch(void);
        float getRoll(void);
        float getYaw(void);

        boolean isNewRegisters(void);

        // int16_t accelCount[3];                                          // Stores the 16-bit signed accelerometer sensor output
        // int16_t gyroCount[3];                                           // Stores the 16-bit signed gyro sensor output
        // int16_t magCount[3];                                            // Stores the 16-bit signed magnetometer sensor output
        // float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};    // Factory mag calibration and mag bias
        // float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};        // Bias corrections for gyro and accelerometer
        // int16_t tempCount;                                              // temperature raw count output
        // float temperature;                                            // Stores the real internal chip temperature in degrees Celsius
        // float SelfTest[6];                                            // holds results of gyro and accelerometer self test

        // float aRes, gRes, mRes;         // scale resolutions per LSB for the sensors

        // float ax, ay, az, gx, gy, gz, mx, my, mz;   // variables to hold latest sensor data values 

        void xxxxxx(void);

    private:

};

#endif // MPU9250LIBV1_H