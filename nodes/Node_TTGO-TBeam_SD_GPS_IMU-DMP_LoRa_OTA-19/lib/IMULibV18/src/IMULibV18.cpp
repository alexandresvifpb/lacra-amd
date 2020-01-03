#include "IMULibV18.h"

int status;
// Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
// MPU9250 IMU(Wire,0x68);
MPU9250 IMU(Wire,MPU_MPU9250_ADDRESS);

// teste

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
  
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// ====== fim de teste

IMULib::IMULib() {}

//======================
// Funcoes Plubic
//======================

// 
// Inputs: nenhum
// Return: 
bool IMULib::begin(void) {

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
    // IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    // setting SRD to 9 for a 100 Hz update rate
    // IMU.setSrd(39);
    IMU.setSrd(19);

    IMUTaskDelayMS = IMU_TASK_DELAY_MS;

    return true;
}

// 
// Inputs: nenhum
// Return: 
void IMULib::read(void) {
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
}

// 
// Inputs: nenhum
// Return: 
bool IMULib::avaliable(void) {
    bool result = newValuesIMU;
    newValuesIMU = false;
    return result;
}

// 
// Inputs: nenhum
// Return: 
String IMULib::getStrDataIMU(uint64_t unixTime) {
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

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
// Inputs: 
// Return: void
void IMULib::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * ( q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * ( q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * ( q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
// Inputs: 
// Return: void
void IMULib::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;   

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    Serial.print("ax: ");
    Serial.println(ax);

    Serial.print("ay: ");
    Serial.println(ay);

    Serial.print("az: ");
    Serial.println(az);

    Serial.print("norm: ");
    Serial.println(norm);

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f) {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    } else {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
 
}

// Retorna o pitch
// Inputs: uint8_t tipo do filtro usado para calcular o valor do pitch
// Return: float com o valor pitch
void IMULib::updateQuartenion(uint8_t filter) {
    if (filter == MADGWICK_FILTER) {
        MadgwickQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
        Serial.println("MadgwickQuaternionUpdate()");
    }
    else {
        MahonyQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
        Serial.println("MahonyQuaternionUpdate()");
    }
}

// Retorna o pitch
// Inputs: uint8_t tipo do filtro usado para calcular o valor do pitch
// Return: float com o valor pitch
float IMULib::getPitch(void) {
    // read();
    // if (filter == 0) MadgwickQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    // else MahonyQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    
    Serial.print("q[1]");
    Serial.println(q[1]);

    float _pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));

    Serial.print("pitch: ");
    Serial.println(_pitch);

    _pitch *= 180.0f / PI;
    return _pitch;
}

// Retorna o pitch
// Inputs: uint8_t tipo do filtro usado para calcular o valor do pitch
// Return: float com o valor pitch
float IMULib::getRoll(void) {
    // read();
    // if (filter == 0) MadgwickQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    // else MahonyQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    
    float _roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    _roll *= 180.0f / PI;
    return _roll;
}

// Retorna o pitch
// Inputs: uint8_t tipo do filtro usado para calcular o valor do pitch
// Return: float com o valor pitch
float IMULib::getYaw(void) {
    // read();
    // if (filter == 0) MadgwickQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    // else MahonyQuaternionUpdate(valuesIMU.accelX, valuesIMU.accelY, valuesIMU.accelZ, valuesIMU.gyroX, valuesIMU.gyroY, valuesIMU.gyroZ, valuesIMU.magX, valuesIMU.magY, valuesIMU.magZ);
    
    float _yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    _yaw *= 180.0f / PI;
    _yaw += 21.33;          // Declination at Areia 2019-12-28	21.55° W
    return _yaw;
}

//======================
// Funcoes Private
//======================

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float IMULib::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}