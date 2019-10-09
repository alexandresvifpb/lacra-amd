#include <Arduino.h>
 
// Serial           
#define MESSAGE_INITIAL                 ("D:\\Users\\alexa\\OneDrive\\doutorado\\2019\\prototipos\\firmwares\\edgenodes\\EdgeNode_EPS32-TTGO_SD_HRSpO2_GPS_IMU_LoRa-11")          // localizacao do projeto
#define DEBUG                           (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa

TaskHandle_t Task0;
TaskHandle_t Task1;

time_t nowTime();
void Task0code( void * pvParameters );
void Task1code( void * pvParameters );

//===========================================
//  Watchdog Timer
#include "esp_system.h"

#define   WATCHDOG_TIME_OUT_SEGUND  (30)    // tempo em segundos para ativar o watchdog

const int wdtTimeout = WATCHDOG_TIME_OUT_SEGUND * 1000;           //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void resetWatchdog(void);

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

//===========================================
//  SD Card
#include "SDCardLibV1.h"

SDCardLib sdCard;

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV3.h"

LoRaNodeLib lora = LoRaNodeLib();

//===========================================
//  GPS NEO6MV2
#include "GPSNeo6MLibV3.h"

GPSNeo6MLib gps;

String strParametersGPS;
boolean isActiveGPS = false;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMUMpu9250LibV3.h"

IMUMpu9255Lib imu = IMUMpu9255Lib();

String strParametersIMU;
boolean isActiveIMU = false;

//===========================================
//  HR & SpO2 MAX30100
#include <Wire.h>
#include "MAX30100_Sensor.h"
// #include "HRSpO2LibV1.h"

HRSpO2 sensor_HRSpO2;
// HRSpO2Lib sensor_HRSpO2;

String strParametersHRSpO2;
boolean isActiveHRSpO2 = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MESSAGE_INITIAL);
    Serial.println();

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                    Task0code,   /* Task function. */
                    "Task0",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task0,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 

    //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task0code: 
void Task0code( void * pvParameters ) {
    Serial.print("Task0 running on core ");
    Serial.println(xPortGetCoreID());

    //===========================================
    //  Watchdog Timer
    timer = timerBegin(0, 80, true);                  //timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
    timerAlarmEnable(timer);                          //enable interrupt
    delay(100);

    //===========================================
    // SD Card
    if (!sdCard.begin())
        Serial.println("Error inicializacao do modulo SDcard");
    else
        Serial.println("SDCard encontrado!");
    delay(100);

    //===========================================
    // LoRa
    if (!lora.begin())
        Serial.println("Error inicializacao do modulo LoRa");
    else
        Serial.println("Modulo LoRa OK!");
    delay(100);

    //===========================================
    // Inicializacao do sensor HR & SpO2
    Serial.println("Inicializando sensor HR & SpO2.....");
    if (!sensor_HRSpO2.begin()) {
        isActiveHRSpO2 = false;
    } else {
        isActiveHRSpO2 = true;
    }

    for(;;) {

        lora.run();

        resetWatchdog();        // reinicializa o watchdog
    
        // HR & SpO2
        if (isActiveHRSpO2) {
            sensor_HRSpO2.run();

            if (sensor_HRSpO2.avaliable()) {
                strParametersHRSpO2 = sensor_HRSpO2.getStrParamHRSpO2(nowTime());
                if (sdCard.isSDCardPresent()) {
                    // sdCard.appendFile(SD, "/Teste.dat", strParametersGPS.c_str());
                    sdCard.appendFile(SD, "/Teste.dat", sdCard.encoderJSon(lora.getLocalAddress(), HRSPO2_TYPE, strParametersHRSpO2).c_str());
                }
                lora.addNewMsg(strParametersHRSpO2, HRSPO2_TYPE);
            }
        }

        resetWatchdog();        // reinicializa o watchdog

        delay(1);       // delay para liberar a prioridade para a proxima thread
    }
}

//Task1code: 
void Task1code( void * pvParameters ) {
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());

    //===========================================
    // Inicializacao do modulo MPU9250
    Serial.println("Inicializando sensor IMU.....");
    if (!imu.begin()) {
        isActiveIMU = false;
    } else {
        isActiveIMU = true;
    }

    //===========================================
    // Inicializacao do sensor GPS
    Serial.println("Inicializando GPS.....");
    if (!gps.begin()) {
        isActiveGPS = false;
    } else {
        gps.requestSync();
        isActiveGPS = true;
    }

    for(;;) {
        // String strSend;
        // strSend = "[";
        // strSend += millis();
        // strSend += "]";
        // lora.send(strSend);
        // lora.addNewMsg(strSend);

        // GPS
        if (isActiveGPS) {
            gps.run();

            if (gps.isAvaliableDatas()) {
                strParametersGPS = gps.getStringDataGPS(nowTime());
                // lora.send(strParametersGPS);
                if (sdCard.isSDCardPresent()) {
                    // sdCard.appendFile(SD, "/Teste.dat", strParametersGPS.c_str());
                    sdCard.appendFile(SD, "/Teste.dat", sdCard.encoderJSon(lora.getLocalAddress(), GPS_TYPE, strParametersGPS).c_str());
                }
                lora.addNewMsg(strParametersGPS, GPS_TYPE);
                // lora.addNewMsg(strParametersGPS, 0);
            }
        }

        // IMU
        if (isActiveIMU) {
            imu.run();
            if (imu.avaliable()) {
                strParametersIMU = imu.getStrDataIMU(nowTime());
                // lora.send(strParametersIMU);
                if (sdCard.isSDCardPresent()) {
                    // sdCard.appendFile(SD, "/Teste.dat", strParametersIMU.c_str());
                    sdCard.appendFile(SD, "/Teste.dat", sdCard.encoderJSon(lora.getLocalAddress(), IMU_TYPE, strParametersIMU).c_str());
                }
                lora.addNewMsg(strParametersIMU, IMU_TYPE);
                // lora.addNewMsg(strParametersIMU, 5);
            } 
        }

        resetWatchdog();        // reinicializa o watchdog

        delay(1);       // delay para liberar a prioridade para a proxima thread
    } 
}

time_t nowTime(void) {
    if (isActiveGPS) {
        return gps.getUnixTime();
    }
    else {
        return millis();
    }
    
}

//===========================================
//  Watchdog Timer
void resetWatchdog(void) {
    timerWrite(timer, 0); //reset timer (feed watchdog)
    return;  
}

void loop() {
}