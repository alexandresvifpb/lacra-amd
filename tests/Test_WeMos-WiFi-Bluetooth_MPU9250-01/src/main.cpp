#include "MPU9250LibV1.h"

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\tests\Test_WeMos-WiFi-Bluetooth_MPU9250-01
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\tests\\Test_WeMos-WiFi-Bluetooth_MPU9250-01")          // localizacao do projeto

MPU9250Lib mpu;

MPU_t dataRegisters;
Quaternion_t dataQ;

int16_t tempCount;                                              // temperature raw count output
float   temperature;                                            // Stores the real internal chip temperature in degrees Celsius

uint32_t delt_t = 0;                        // used to control display output rate
uint32_t count = 0, sumCount = 0;           // used to control display output rate
float pitch, yaw, roll;
// float deltat = 0.0f, sum = 0.0f;            // integration interval for both filter schemes
float sum = 0.0f;            // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;   // used to calculate integration interval
uint32_t Now = 0;                           // used to calculate integration interval

void setup() {
    Serial.begin(115200);
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    // Wire.begin();       // inicializa a porta I2C

    mpu.begin();        // inicializa o MPU9250
}

void loop() {

    if (mpu.isNewRegisters()) {                 // verifica se novos dados estao disponivel
        dataRegisters = mpu.getRegisters();     // pega os dados disponiveis
    }

    Now = micros();
    // deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    mpu.setDeltat(((Now - lastUpdate)/1000000.0f)); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    // sum += deltat; // sum for averaging filter update rate
    sum += mpu.getDeltat(); // sum for averaging filter update rate
    sumCount++;
  
    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!  
    // Pass gyro rate as rad/s
    //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    //  mpu.madgwickQuaternionUpdate(dataRegisters.accelX, dataRegisters.accelY, dataRegisters.accelZ, dataRegisters.gyroX*PI/180.0f, dataRegisters.gyroY*PI/180.0f, dataRegisters.gyroZ*PI/180.0f, dataRegisters.magX, dataRegisters.magY, dataRegisters.magZ);
    // MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
    mpu.mahonyQuaternionUpdate(dataRegisters.accelX, dataRegisters.accelY, dataRegisters.accelZ, dataRegisters.gyroX*PI/180.0f, dataRegisters.gyroY*PI/180.0f, dataRegisters.gyroZ*PI/180.0f, dataRegisters.magX, dataRegisters.magY, dataRegisters.magZ);
    dataQ = mpu.getQuaternion();

    if (!AHRS) {
        delt_t = millis() - count;
        if(delt_t > 500) {

            if(SerialDebug) {
                // Print acceleration values in milligs!
                Serial.print("X-acceleration: "); Serial.print(1000*dataRegisters.accelX); Serial.print(" mg ");
                Serial.print("Y-acceleration: "); Serial.print(1000*dataRegisters.accelY); Serial.print(" mg ");
                Serial.print("Z-acceleration: "); Serial.print(1000*dataRegisters.accelZ); Serial.println(" mg ");
 
                // Print gyro values in degree/sec
                Serial.print("X-gyro rate: "); Serial.print(dataRegisters.gyroX, 3); Serial.print(" degrees/sec "); 
                Serial.print("Y-gyro rate: "); Serial.print(dataRegisters.gyroY, 3); Serial.print(" degrees/sec "); 
                Serial.print("Z-gyro rate: "); Serial.print(dataRegisters.gyroZ, 3); Serial.println(" degrees/sec"); 
    
                // Print mag values in degree/sec
                Serial.print("X-mag field: "); Serial.print(dataRegisters.magX); Serial.print(" mG "); 
                Serial.print("Y-mag field: "); Serial.print(dataRegisters.magY); Serial.print(" mG "); 
                Serial.print("Z-mag field: "); Serial.print(dataRegisters.magZ); Serial.println(" mG"); 
 
                tempCount = mpu.readTempData();  // Read the adc values
                temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
                // Print temperature in degrees Centigrade      
                Serial.print("Temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
            }
    
            count = millis();
        }
    } else {
      
        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = millis() - count;
        if (delt_t > 500) { // update LCD once per half-second independent of read rate

            if(SerialDebug) {
                Serial.print("ax = "); Serial.print((int)1000*dataRegisters.accelX);  
                Serial.print(" ay = "); Serial.print((int)1000*dataRegisters.accelY); 
                Serial.print(" az = "); Serial.print((int)1000*dataRegisters.accelZ); Serial.println(" mg");

                Serial.print("gx = "); Serial.print( dataRegisters.gyroX, 2); 
                Serial.print(" gy = "); Serial.print( dataRegisters.gyroY, 2); 
                Serial.print(" gz = "); Serial.print( dataRegisters.gyroZ, 2); Serial.println(" deg/s");
                
                Serial.print("mx = "); Serial.print( (int)dataRegisters.magX ); 
                Serial.print(" my = "); Serial.print( (int)dataRegisters.magY ); 
                Serial.print(" mz = "); Serial.print( (int)dataRegisters.magZ ); Serial.println(" mG");
    
                Serial.print("q0 = "); Serial.print(dataQ.q0);      // q[0]
                Serial.print(" qx = "); Serial.print(dataQ.qx);     // q[1]
                Serial.print(" qy = "); Serial.print(dataQ.qy);     // q[2]
                Serial.print(" qz = "); Serial.println(dataQ.qz);   // q[3]
            }               
    
            // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
            // In this coordinate system, the positive z-axis is down toward Earth. 
            // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
            // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
            // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
            // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
            // applied in the correct order which for this configuration is yaw, pitch, and then roll.
            // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
            yaw   = atan2(2.0f * (dataQ.qx * dataQ.qy + dataQ.q0 * dataQ.qz), dataQ.q0 * dataQ.q0 + dataQ.qx * dataQ.qx - dataQ.qy * dataQ.qy - dataQ.qz * dataQ.qz);   
            pitch = -asin(2.0f * (dataQ.qx * dataQ.qz - dataQ.q0 * dataQ.qy));
            roll  = atan2(2.0f * (dataQ.q0 * dataQ.qx + dataQ.qy * dataQ.qz), dataQ.q0 * dataQ.q0 - dataQ.qx * dataQ.qx - dataQ.qy * dataQ.qy + dataQ.qz * dataQ.qz);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI; 
            yaw   += 21.73; // Declination at Areia, Paraiba is -21 degrees 43 minutes and 48 seconds on 2019-12-29
            roll  *= 180.0f / PI;
     
            if(SerialDebug) {
                Serial.print("Yaw, Pitch, Roll: ");
                Serial.print(yaw, 2);
                Serial.print(", ");
                Serial.print(pitch, 2);
                Serial.print(", ");
                Serial.println(roll, 2);
    
                Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
            }

            count = millis(); 
            sumCount = 0;
            sum = 0;    
        }
    }
}