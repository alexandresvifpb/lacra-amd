#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-TBeam_GPS_IMU-DMP_SD_LoRa_WiFi-OTA-WS-21
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-TBeam_GPS_IMU-DMP_SD_LoRa_WiFi-OTA-WS-24.00")          // localizacao do projeto
#define MAIN_DEBUG                      (true)          // Variable that enables (1) or disables (0) the sending of data by serial to debug the program
#define MAIN_CORE_0                     (0)             // Defined the core to be used
#define MAIN_CORE_1                     (1)             
#define MAIN_WATCHDOC_TIME_OUT          (600)           // Watchdog wait time to restart the process in seconds

TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdWiFiOTA;
TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdIMU;

void TaskLoRa( void * pvParameters );
void TaskWiFiOTA( void * pvParameters );
void TaskSDCard( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskIMU( void * pvParameters );

//===========================================
//  ESP32 Util
#include "ESP32NodeLibV22.h"

ESP32NodeLib module;
String addressIdESP = module.getIdModule();
uint16_t bootSequence = -1;
boolean dateTimeValid = false;

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV21.h"

LoRaNodeLib lora;
boolean enabledLoRa = false;

//===========================================
//  Library to implement functionality using the ESP32 WiFi module
#include "WiFiNodeOTALibV5.h"

WiFiOTALib wifi;
boolean enabledWiFiOTA = false;
boolean isConnectedSTA = false;          // Connected wifi network?
boolean isConnectedOTA = false;          // Active OTA service?
boolean isConnectedClientWS = false;     // One or more clients connected to the webserver?

//===========================================
//  SD Card
#include "SDCardLibV20.h"

SDCardLib sdCard;
boolean enabledSDCard = false;

//===========================================
//  GPS NEO6MV2
#include "GPSLibV21.h"

GPSLib gps;
boolean enabledGPS = false;
String stringDatasGPS = "";
uint32_t recordCounterGPS = 0;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMULibV21.h"

IMULib sensorIMU;
boolean enabledIMU = false;
String stringDatasIMU = "";
uint32_t recordCounterIMU = 0;

void setup() {
    // Serial Initialization
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    EEPROM.begin(2);
    bootSequence = module.getBootSequence();
    Serial.print("Boot Sequence: ");
    Serial.println(bootSequence);

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 0
    xTaskCreatePinnedToCore(
                    TaskLoRa,       // Function with the code that implements the Task
                    "TaskLoRa",     // Task name
                    4096,           // Stack size to be allocated when creating the Task
                    NULL,           // Task input parameters
                    1,              // Task priority
                    &TaskIdLoRa,    // Reference to accompany the Task
                    MAIN_CORE_0);   // Core on which the Task will run (0 or 1 for ESP32)
    delay(100);                     // Delay for next command

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 1
    xTaskCreatePinnedToCore(
                    TaskWiFiOTA,    // Function with the code that implements the Task
                    "TaskSDCard",   // Task name
                    4096,           // Stack size to be allocated when creating the Task
                    NULL,           // Task input parameters
                    1,              // Task priority
                    &TaskIdWiFiOTA, // Reference to accompany the Task
                    MAIN_CORE_1);   // Core on which the Task will run (0 or 1 for ESP32)
    delay(100);                     // Delay for next command

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 1
    xTaskCreatePinnedToCore(
                    TaskSDCard,     // Function with the code that implements the Task
                    "TaskSDCard",   // Task name
                    4096,           // Stack size to be allocated when creating the Task
                    NULL,           // Task input parameters
                    1,              // Task priority
                    &TaskIdSDCard,  // Reference to accompany the Task
                    MAIN_CORE_1);   // Core on which the Task will run (0 or 1 for ESP32)
    delay(100);                     // Delay for next command

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 1
    xTaskCreatePinnedToCore(
                    TaskGPS,        // Function with the code that implements the Task
                    "TaskGPS",      // Task name
                    4096,           // Stack size to be allocated when creating the Task
                    NULL,           // Task input parameters
                    1,              // Task priority
                    &TaskIdGPS,     // Reference to accompany the Task
                    MAIN_CORE_1);   // Core on which the Task will run (0 or 1 for ESP32)
    delay(100);                     // Delay for next command

    // Creates a Task that will execute the TaskLoRa () function, with priority 1 and running in the nucleus 0
    xTaskCreatePinnedToCore(
                    TaskIMU,        // Function with the code that implements the Task
                    "TaskIMU",      // Task name
                    4096,           // Stack size to be allocated when creating the Task
                    NULL,           // Task input parameters
                    1,              // Task priority
                    &TaskIdIMU,     // Reference to accompany the Task
                    MAIN_CORE_0);   // Core on which the Task will run (0 or 1 for ESP32)
    delay(100);                     // Delay for next command

    // Enables the watchdog with a 15-second timeout
    esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
}

// Task for the LoRa radio module
void TaskLoRa( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // LoRa
    enabledLoRa = lora.begin();;
    ( !enabledLoRa ) ? Serial.println("Error initializing the LoRa module") : Serial.println("LoRa module OK!");

    // Mandatory infinite loop to keep the Task running
    while( true ) {
        lora.run();

        if ( enabledLoRa ) {
            if ( lora.waitForNextSend() ) {
                recordFormatLoRa_t newRecordLoRa;
                newRecordLoRa.id = addressIdESP;
                // newRecordLoRa.type = TYPE_GPS;
                newRecordLoRa.type = 9;
                newRecordLoRa.payload = String((unsigned long)module.getUnixTimeNow());
                newRecordLoRa.payload += ',';
                newRecordLoRa.payload += bootSequence;
                newRecordLoRa.payload += ',';
                ( sdCard.isSDCardPresent() ) ? newRecordLoRa.payload += "1": newRecordLoRa.payload += "0";
                newRecordLoRa.payload += ',';
                newRecordLoRa.payload += recordCounterGPS;
                newRecordLoRa.payload += ',';
                newRecordLoRa.payload += recordCounterIMU;
                lora.addRecordToSend(newRecordLoRa);
            }
        }

        esp_task_wdt_reset();                                   // Reset watchdog counter 
        vTaskDelay(pdMS_TO_TICKS(lora.tsLoRaTaskDelayMS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
    }
}

// Task for the WiFi module
void TaskWiFiOTA( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Initializes the ESP32 wifi module
    enabledWiFiOTA = wifi.begin();
    ( !enabledWiFiOTA ) ? Serial.println("Error initializing the WiFi") : Serial.println("WiFi OK!");

    wifi.setMACAddress(addressIdESP);

    // Mandatory infinite loop to keep the Task running
    while(true) {
        isConnectedSTA = wifi.isConnectedSTA();

        if ( !isConnectedSTA ) {
            isConnectedSTA = wifi.initSTA();
        }
        
        if ( isConnectedSTA && !isConnectedOTA ) {
            isConnectedOTA = wifi.activeOTA();
        }

        if ( isConnectedOTA ) {
            // Handle is a descriptor that references variables in the memory block
            // It is used as a "guide" so that ESP can communicate with the computer over the network
            ArduinoOTA.handle();
        }

        if ( ( isConnectedSTA ) ) {
            if ( wifi.listenForClientsWS() == 1) {
                isConnectedClientWS = true;
            }

            if ( wifi.availableClientWS() == 0) {
                isConnectedClientWS = false;
            }
        }

        // Checks if data is checked by the WebServer            
        if ( isConnectedClientWS ) {
            if ( wifi.availableRecvDataClientWS() ) {
                String recvCommandWebSocket = wifi.readClientWS();

                Serial.print("RECV WEB SOCKET: ");
                Serial.println(recvCommandWebSocket);

            }
        }

        if ( wifi.availableClientWS() > 0 ) {
            Serial.println("WIFI_INFO: Client connected");
        }

        esp_task_wdt_reset();                                           // Reset watchdog counter
        vTaskDelay(pdMS_TO_TICKS(wifi.tsWiFiNodeOTATaskDelayMS));       // Pause Tesk and release the nucleus for the next Tesk in the priority queue
    }
}

// Task for the SDCard module
void TaskSDCard( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Initializes the SD Card module
    enabledSDCard = sdCard.begin();
    ( !enabledSDCard ) ? Serial.println("SD Card module initialization error") : Serial.println("SD Card Module OK!");

    // Mandatory infinite loop to keep the Task running
    while(true)
    {
        if ( enabledSDCard ) {
            sdCard.run();
        }

        esp_task_wdt_reset();                                       // Reset watchdog counter
        vTaskDelay(pdMS_TO_TICKS(sdCard.tsSDCardTaskDelayMS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
    }
}

// Task for the GPS module
void TaskGPS( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Initializes the ESP32 wifi module
    enabledGPS = gps.begin();
    ( !enabledGPS ) ? Serial.println("GPS module initialization error") : Serial.println("GPS module OK!");
    gps_t currentDatasGPS;

    // Mandatory infinite loop to keep the Task running
    while(true)
    {
        // Serial.println(__LINE__);

        if ( enabledGPS ) {
            gps.run();

            // Serial.println(__LINE__);

            if ( !dateTimeValid ) {
                // Serial.println(__LINE__);
                if ( gps.isDateTimeValid() ) {
                    // Serial.println(__LINE__);
                    module.setUnixTime(gps.getUnixTimeNow());
                    dateTimeValid = true;
                }
            }

            if ( module.getUnixTimeNow() != gps.getUnixTimeNow() ) {
                dateTimeValid = false;
            }

            if ( gps.isLocationValid() && gps.isLocationUpdate() ) {
                recordFormatSDCard_t newRecordGPS;
                time_t epochTimeNow = module.getUnixTimeNow();
                stringDatasGPS = gps.getStringDatas(epochTimeNow, bootSequence);

                newRecordGPS.id = addressIdESP;
                newRecordGPS.bootSequence = bootSequence;
                newRecordGPS.type = TYPE_GPS;
                newRecordGPS.payload = stringDatasGPS;

                if ( sdCard.isSDCardPresent() ) sdCard.addRecord(newRecordGPS);

                if ( wifi.availableClientWS() ) wifi.printClientWS(stringDatasGPS);

                recordCounterGPS++;

                Serial.println(stringDatasGPS);
            }
        }

        esp_task_wdt_reset();                                   // Reset watchdog counter
        vTaskDelay(pdMS_TO_TICKS(gps.tsGPSTaskDelayMS));        // Pause Tesk and release the nucleus for the next Tesk in the priority queue
    }
}

// Task for the GPS module
void TaskIMU( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Initializes the MPU9250 module
    enabledIMU = sensorIMU.begin();
    ( !enabledIMU ) ? Serial.println("Error initializing the IMU module (MPU9250)") : Serial.println("IMU module (MPU9250) OK!");

    // Mandatory infinite loop to keep the Task running
    while(true)
    {
        if ( enabledIMU ) {
            if ( sensorIMU.avaliable() ) {
                if ( sensorIMU.updateDatas() ) {

                    recordFormatSDCard_t newRecordIMU;
                    time_t epochTimeNow = module.getUnixTimeNow();
                    stringDatasIMU = sensorIMU.getStringDatas(epochTimeNow, bootSequence);

                    newRecordIMU.id = addressIdESP;
                    newRecordIMU.bootSequence = bootSequence;
                    newRecordIMU.type = TYPE_IMU;
                    newRecordIMU.payload = stringDatasIMU;

                    if ( sdCard.isSDCardPresent() ) sdCard.addRecord(newRecordIMU);

                    recordCounterIMU++;

                    Serial.println(stringDatasIMU);
                }
            }
        }

        esp_task_wdt_reset();                                       // Reset watchdog counter
        vTaskDelay(pdMS_TO_TICKS(sensorIMU.tsIMUTaskDelayMS));      // Pause Tesk and release the nucleus for the next Tesk in the priority queue
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}