#include "IMUMpu9250LibV3.h"

int status;
// Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
MPU9250 IMU(Wire,0x68);

IMUMpu9255Lib::IMUMpu9255Lib() {}

bool IMUMpu9255Lib::begin(void) {

    status = IMU.begin();

    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        // while(1) {}

        return false;
    }

    // setting the accelerometer full scale range to +/-8G 
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

    return true;
}

void IMUMpu9255Lib::run(void) {
    if (millis() - tsLastReport > ACGYMG_REPORTING_PERIOD_MS) {

        // read the sensor
        if (IMU.readSensor() > 0) {

            // Serial.println(IMU.getAccelX_mss);

            if ((abs(valuesIMU.accelX - IMU.getAccelX_mss()) > dataAccelDif) ||
                (abs(valuesIMU.accelY - IMU.getAccelY_mss()) > dataAccelDif) ||
                (abs(valuesIMU.accelZ - IMU.getAccelZ_mss()) > dataAccelDif) )  {

            newValuesIMU = true;

                }


            if (newValuesIMU) {
                valuesIMU.accelX = IMU.getAccelX_mss();
                valuesIMU.accelY = IMU.getAccelY_mss();
                valuesIMU.accelZ = IMU.getAccelZ_mss();

                valuesIMU.gyroX = IMU.getGyroX_rads();
                valuesIMU.gyroY = IMU.getGyroY_rads();
                valuesIMU.gyroZ = IMU.getGyroZ_rads();

                valuesIMU.magX = IMU.getMagX_uT();
                valuesIMU.magY = IMU.getMagY_uT();
                valuesIMU.magZ = IMU.getMagZ_uT();

                valuesIMU.temperature_c = IMU.getTemperature_C();
            }
        }

        tsLastReport = millis();
    }
}

bool IMUMpu9255Lib::avaliable(void) {
    bool result = newValuesIMU;
    newValuesIMU = false;
    return result;
}

String IMUMpu9255Lib::getStrDataIMU(uint64_t unixTime) {
    String result = "[";
    // result += IMU_TYPE;
    // result += ",";
    result += (unsigned long)unixTime;
    result += ",";

    // Serial.println(valuesIMU.accelX, 6);

    result += String(valuesIMU.accelX,6);
    result += ",";
    result += String(valuesIMU.accelY,6);
    result += ",";
    result += String(valuesIMU.accelZ,6);
    result += ",";
    result += String(valuesIMU.gyroX,6);
    result += ",";
    result += String(valuesIMU.gyroY,6);
    result += ",";
    result += String(valuesIMU.gyroZ,6);
    result += ",";
    result += String(valuesIMU.magX,6);
    result += ",";
    result += String(valuesIMU.magY,6);
    result += ",";
    result += String(valuesIMU.magZ,6);
    result += ",";
    result += String(valuesIMU.magDirection,6);
    // result += ",";
    // result += String(valuesIMU.pressure,6);
    // result += ",";
    // result += String(valuesIMU.temperature_c,6);
    result += ",";
    result += getVBat();
    result += "]";
    return result;
}

//======================
// Private
//======================

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float IMUMpu9255Lib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}