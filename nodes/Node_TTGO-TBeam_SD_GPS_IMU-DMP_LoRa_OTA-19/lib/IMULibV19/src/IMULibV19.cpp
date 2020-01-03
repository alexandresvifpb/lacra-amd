#include "IMULibV19.h"

MPU9250_DMP imu;

boolean newDatasAvaliable = false;

IMULib::IMULib() {}

//======================
// Funcoes Plubic
//======================

// 
// Inputs: void
// Return: boolean
boolean IMULib::begin(void)
{

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS) return false;

    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    // Enable all sensors:
    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at 
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    imu.setCompassSampleRate(10); // Set mag rate to 10Hz

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                 DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 10); // Set DMP FIFO rate to 10 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the 
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

    return true;
}

// 
// Inputs: void
// Return: boolean
boolean IMULib::avaliable(void) 
{
    newDatasAvaliable = imu.fifoAvailable();
    return newDatasAvaliable;
}

// 
// Inputs: void
// Return: boolean
boolean IMULib::updateDatas(void) 
{
    if (newDatasAvaliable)
    {
        newDatasAvaliable = false;
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS)
        {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles();
        }
        else return false;      // se não tiver dados disponivel na FIFO retorna false

        // dataReady() checks to see if new accel/gyro data
        // is available. It will return a boolean true or false
        // (New magnetometer data cannot be checked, as the library
        //  runs that sensor in single-conversion mode.)
        if ( imu.dataReady() )
        {
            // Call update() to update the imu objects sensor data.
            // You can specify which sensors to update by combining
            // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
            // UPDATE_TEMPERATURE.
            // (The update function defaults to accel, gyro, compass,
            //  so you don't have to specify these values.)
            imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        }
        return true;            // dados na FIFO disponivel
    } 
    else
    {
        return false;
    }
    
}

// 
// Inputs: void
// Return: IMU_t
IMU_t IMULib::getIMU(void)
{
    IMU_t datasIMU;

    datasIMU.accelX = imu.calcAccel(imu.ax);
    datasIMU.accelY = imu.calcAccel(imu.ay);
    datasIMU.accelZ = imu.calcAccel(imu.az);

    datasIMU.gyroX = imu.calcGyro(imu.gx);
    datasIMU.gyroY = imu.calcGyro(imu.gy);
    datasIMU.gyroZ = imu.calcGyro(imu.gz);

    datasIMU.magX = imu.calcMag(imu.mx);
    datasIMU.magY = imu.calcMag(imu.my);
    datasIMU.magZ = imu.calcMag(imu.mz);

    // Serial.print("imu.ax: "), Serial.println(imu.calcAccel(imu.ax));

    return datasIMU;
}

// 
// Inputs: void
// Return: Quaternion_t
Quaternion_t IMULib::getQuaternion(void)
{
    Quaternion_t datasQuaternion;

    datasQuaternion.q0 = imu.calcQuat(imu.qw);
    datasQuaternion.q1 = imu.calcQuat(imu.qx);
    datasQuaternion.q2 = imu.calcQuat(imu.qy);
    datasQuaternion.q3 = imu.calcQuat(imu.qz);

    return datasQuaternion;
}

// 
// Inputs: void
// Return: EulerAngles_t
EulerAngles_t IMULib::getEulerAngles(void)
{
    EulerAngles_t datasEulerAngles;

    datasEulerAngles.Pitch = imu.pitch;
    datasEulerAngles.Roll = imu.roll;
    datasEulerAngles.Yam = imu.yaw;

    return datasEulerAngles;
}

// 
// Inputs: uint64_t; IMU_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, IMU_t datasIMU)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrIMU(datasIMU);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, Quaternion_t datasQuaternion)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrQuaternion(datasQuaternion);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, EulerAngles_t datasEulerAngles)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrEulerAngles(datasEulerAngles);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; IMU_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, IMU_t datasIMU, Quaternion_t datasQuaternion)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrIMU(datasIMU);
    result += ",";
    result += getStrQuaternion(datasQuaternion);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; IMU_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, IMU_t datasIMU, EulerAngles_t datasEulerAngles)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrIMU(datasIMU);
    result += ",";
    result += getStrEulerAngles(datasEulerAngles);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; IMU_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, Quaternion_t datasQuaternion, EulerAngles_t datasEulerAngles)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrQuaternion(datasQuaternion);
    result += ",";
    result += getStrEulerAngles(datasEulerAngles);
    result += "]";
    return result;
}

// 
// Inputs: uint64_t; IMU_t; Quaternion_t
// Return: String
String IMULib::getStrDatas(uint64_t unixTime, IMU_t datasIMU, Quaternion_t datasQuaternion, EulerAngles_t datasEulerAngles)
{
    String result = "[";
    result += (unsigned long)unixTime;
    result += ",";
    result += getStrIMU(datasIMU);
    result += ",";
    result += getStrQuaternion(datasQuaternion);
    result += ",";
    result += getStrEulerAngles(datasEulerAngles);
    result += "]";
    return result;
}


//==============================

// 
// Inputs: uint64_t; IMU_t
// Return: String
String IMULib::getStrIMU(IMU_t datasIMU)
{
    String result;
    result += String(datasIMU.accelX,4);
    result += ",";
    result += String(datasIMU.accelY,4);
    result += ",";
    result += String(datasIMU.accelZ,4);
    result += ",";
    result += String(datasIMU.gyroX,4);
    result += ",";
    result += String(datasIMU.gyroY,4);
    result += ",";
    result += String(datasIMU.gyroZ,4);
    result += ",";
    result += String(datasIMU.magX,4);
    result += ",";
    result += String(datasIMU.magY,4);
    result += ",";
    result += String(datasIMU.magZ,4);
    return result;
}

// 
// Inputs: uint64_t; IMU_t
// Return: String
String IMULib::getStrQuaternion(Quaternion_t datasQuaternion)
{
    String result;
    result += String(datasQuaternion.q0,4);
    result += ",";
    result += String(datasQuaternion.q1,4);
    result += ",";
    result += String(datasQuaternion.q2,4);
    result += ",";
    result += String(datasQuaternion.q3,4);
    return result;
}

// 
// Inputs: uint64_t; IMU_t
// Return: String
String IMULib::getStrEulerAngles(EulerAngles_t datasEulerAngles)
{
    String result;
    result += String(datasEulerAngles.Yam,4);
    result += ",";
    result += String(datasEulerAngles.Pitch,4);
    result += ",";
    result += String(datasEulerAngles.Roll,4);
    return result;
}