#include "IMULibV21.h"

MPU9250_DMP imu;

boolean newDatasAvaliable = false;

IMULib::IMULib() {}

// 
boolean IMULib::begin(void) {

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
    imu.setAccelFSR(8); // Set accel to +/-2g
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
boolean IMULib::avaliable(void) {
    newDatasAvaliable = imu.fifoAvailable();
    return newDatasAvaliable;
}

// 
boolean IMULib::updateDatas(void) {
    if (newDatasAvaliable) {
        newDatasAvaliable = false;
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS) {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles();
        }
        else return false;      // se não tiver dados disponivel na FIFO retorna false

        // dataReady() checks to see if new accel/gyro data
        // is available. It will return a boolean true or false
        // (New magnetometer data cannot be checked, as the library
        //  runs that sensor in single-conversion mode.)
        if ( imu.dataReady() ) {
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
    else {
        return false;
    }
}

// 
IMU_t IMULib::getIMU(void) {
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

    return datasIMU;
}

// 
Quaternion_t IMULib::getQuaternion(void) {
    Quaternion_t datasQuaternion;

    datasQuaternion.q0 = imu.calcQuat(imu.qw);
    datasQuaternion.q1 = imu.calcQuat(imu.qx);
    datasQuaternion.q2 = imu.calcQuat(imu.qy);
    datasQuaternion.q3 = imu.calcQuat(imu.qz);

    return datasQuaternion;
}

// 
EulerAngles_t IMULib::getEulerianAngle(Quaternion_t quaternion) {
    EulerAngles_t eulerAngles;

    // double ysqr = y * y;
    double ysqr = quaternion.q2 * quaternion.q2;

    // roll (x-axis rotation)

    // double t0 = +2.0 * (w * x + y * z);
    // double t1 = +1.0 - 2.0 * (x * x + ysqr);
    // roll = atan2(t0, t1);

    double t0 = +2.0 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3);
    double t1 = +1.0 - 2.0 * (quaternion.q1 * quaternion.q1 + ysqr);
    eulerAngles.Roll = atan2(t0, t1) * (180.0 / PI);

    // pitch (y-axis rotation)

    // double t2 = +2.0 * (w * y - z * x);
    // t2 = t2 > 1.0 ? 1.0 : t2;
    // t2 = t2 < -1.0 ? -1.0 : t2;
    // pitch = asin(t2);

    double t2 = +2.0 * (quaternion.q0 * quaternion.q2 - quaternion.q3 * quaternion.q1);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    eulerAngles.Pitch = asin(t2) * (180.0 / PI);

    // yaw (z-axis rotation)

    // double t3 = +2.0 * (w * z + x * y);
    // double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    // yaw = atan2(t3, t4);

    double t3 = +2.0 * (quaternion.q0 * quaternion.q3 + quaternion.q1 * quaternion.q2);
    double t4 = +1.0 - 2.0 * (ysqr + quaternion.q3 * quaternion.q3);  
    eulerAngles.Yaw = atan2(t3, t4) * (180.0 / PI);

    return eulerAngles;
}

// 
String IMULib::getStringDatas(uint64_t unixTime, uint16_t bootsequence) {
    IMU_t datasIMU = getIMU();
    Quaternion_t datasQuaternion = getQuaternion();
    EulerAngles_t datasEulerAngles = getEulerianAngle(datasQuaternion);

    String result = String((unsigned long)unixTime);
    result += ",";
    result += bootsequence;
    result += ",";
    result += getStrIMU(datasIMU);
    result += ",";
    result += getStrQuaternion(datasQuaternion);
    result += ",";
    result += getStrEulerAngles(datasEulerAngles);
    return result;
}

// 
String IMULib::getStrIMU(IMU_t datasIMU) {
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
String IMULib::getStrQuaternion(Quaternion_t datasQuaternion) {
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
String IMULib::getStrEulerAngles(EulerAngles_t datasEulerAngles) {
    String result;
    result += String(datasEulerAngles.Yaw,4);
    result += ",";
    result += String(datasEulerAngles.Pitch,4);
    result += ",";
    result += String(datasEulerAngles.Roll,4);
    return result;
}