// #include <Arduino.h>
#include "ESP32LibV17.h"
#include "esp_task.h"
#include "esp_task_wdt.h"

//===========================================
//  ESP32 Util

ESP32Lib esp32util = ESP32Lib();
String addressIdESP = esp32util.getMacAddress();

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-TBeam_SD_GPS_IMU_LoRa-17
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-TBeam_SD_GPS_IMU_LoRa-17")          // localizacao do projeto
#define MAIN_DEBUG                      (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (600)            // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdIMU;
TaskHandle_t TaskId5Min;

void TaskLoRa( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskSDCard( void * pvParameters );
void TaskIMU( void * pvParameters );
void Task5Min( void * pvParameters );

//===========================================
//  SD Card
#include "SDCardLibV17.h"

SDCardLib sdCard;
boolean sdCardPresent = false;

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV17.h"

LoRaNodeLib lora = LoRaNodeLib();
bool isWorking_LoRa = false;

//===========================================
//  GPS NEO6MV2
#include "GPSLibV17.h"

GPSLib gps;

boolean isWorking_GPS = false;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMULibV17.h"

IMULib imu = IMULib();

String strParametersIMU;
boolean isActiveIMU = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskSDCard,     // Funcao com o codigo que implenta a Task
                    "TaskSDCard",   // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdSDCard,  // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 1
    xTaskCreatePinnedToCore(
                    TaskLoRa,       // Funcao com o codigo que implenta a Task
                    "TaskLoRa",     // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdLoRa,    // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskGPS,        // Funcao com o codigo que implenta a Task
                    "TaskGPS",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdGPS,     // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskIMU,        // Funcao com o codigo que implenta a Task
                    "TaskIMU",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdIMU,     // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    Task5Min,        // Funcao com o codigo que implenta a Task
                    "Task5Min",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskId5Min,     // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    // Habilitamos o watchdog com timeout de 15 segundos
    esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
}

// Funcao que implementa a Task para o SDCard
// Inputs: nenhum
// Return: nenhum
void TaskSDCard( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // SD Card
    sdCardPresent = sdCard.begin();
    if (!sdCardPresent)
        Serial.println("Error inicializacao do modulo SDcard");
    else
        Serial.println("SDCard encontrado!");

    uint16_t sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (sdCardPresent) {
        if (sdCardTimeLoopSerial) sdCardTimeLoopSerial--;
        else {
            Serial.println(SD_DEBUG_MESSAGE);
            sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;
        }
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(sdCard.sdTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para o modulo de radio LoRa
// Inputs: nenhum
// Return: nenhum
void TaskLoRa( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // LoRa
    isWorking_LoRa = lora.begin();;
    if (!isWorking_LoRa)
        Serial.println("Error inicializacao do modulo LoRa");
    else
        Serial.println("Modulo LoRa OK!");

    uint16_t loraTimeLoopSerial = lora.loraDebugTimeSerial;

    // lora.requestSyncTime();
        
    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        lora.requestSyncTime();

        if (loraTimeLoopSerial) loraTimeLoopSerial--;
        else {
            Serial.println(LORA_DEBUG_MESSAGE);
            loraTimeLoopSerial = lora.loraDebugTimeSerial;
        }

        uint8_t _cont = 10;

        while (esp32util.avaliableSendMessage() && _cont) {

            String strMessageSend = esp32util.getNextSendMessage();

            if (lora.loraDebugSend) Serial.println("---> LoraSend");
            if (lora.loraDebugSend) Serial.println(strMessageSend);

            lora.send(strMessageSend);
            _cont--;
            if (sdCard.isSDCardPresent()) {
                // sdCard.appendFile(SD, "/Teste.dat", strMessageSend.c_str());
                String filename = "/Data_LoRa_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                sdCard.appendFile(SD, filename.c_str(), strMessageSend.c_str());
            }

        }

        // tratar comando recebidos pelo LoRa
        if (lora.isReceived()) {
            messageLoRa_t msgRecvLoRa = lora.getMsgRecv();

            // Switch para tratamento dos comando chegado pelo radio LoRa
            switch (msgRecvLoRa.type) {
            case COMMAND_EPOCH_TIME:        // 10
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    lora.setEpochTime(atol(strPayload.c_str()));
                }
                break;
            
            case COMMAND_IMU_INTERVAL:        // 20
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    // lora.setEpochTime(atol(strPayload.c_str()));
                    esp32util.min5TaskDelayMS = atol(strPayload.c_str());
                }
                break;
            
            case COMMAND_SEND_DATA_LORA:        // 21
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    if (atol(strPayload.c_str())) imu.sendDataForLora = true;
                    else imu.sendDataForLora = false;
                }
                break;
            
            case COMMAND_GPS_INTERVAL:        // 30
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    // lora.setEpochTime(atol(strPayload.c_str()));
                    gps.gpsTaskDelayMS = atol(strPayload.c_str());
                }
                break;
            
            case COMMAND_LORA_INTERVAL:        // 40
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    // lora.setEpochTime(atol(strPayload.c_str()));
                    lora.loraTaskDelayMS = atol(strPayload.c_str());
                }
                break;
            
            case COMMAND_SDCARD_INTERVAL:        // 50
                {
                    String strPayload = msgRecvLoRa.payload;
                    strPayload.replace("[","");
                    strPayload.replace("]","");
                    // lora.setEpochTime(atol(strPayload.c_str()));
                    sdCard.sdTaskDelayMS = atol(strPayload.c_str());
                }
                break;
            
            default:
                break;
            }
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog 
        vTaskDelay(pdMS_TO_TICKS(lora.loraTaskDelayMS));    // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do GPS
// Inputs: nenhum
// Return: nenhum
void TaskGPS( void * pvParameters ) {
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task
    // disableCore0WDT();              // Desabilita o watchdogtime de hardware do core 0

    //===========================================
    // Inicializacao do modulo GPS
    isWorking_GPS = gps.begin();;
    if (isWorking_GPS)
        gps.requestSync();

    uint16_t gpsTimeLoopSerial = gps.gpsDebugTimeSerial;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        esp_task_wdt_reset();                               // Reseta o contador do watchdog

        if (gpsTimeLoopSerial) gpsTimeLoopSerial--;
        else {
            Serial.println(GPS_DEBUG_MESSAGE);
            gpsTimeLoopSerial = gps.gpsDebugTimeSerial;
        }

        if (gps.gpsDebugGetData) Serial.println("---> GPS");

        if (gps.read() && gps.gpsDebugGetData) Serial.println("---> GPS Read");

        if (gps.avaliable()) {
            if (gps.gpsDebugGetData) Serial.println("---> GPS avaliable");
            // String strDataGPS = gps.getDataString(gps.getEpochTime());
            // String strDataGPS = gps.getDataString(esp32util.getEpochTime());
            String strDataGPS = gps.getDataString(now());
            if (gps.gpsDebugGetData) Serial.println(strDataGPS);
            esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS));
        }

        if (gps.gpsDebugGetData) Serial.println(gps.gpsTaskDelayMS);
        if (gps.gpsDebugGetData) Serial.println("GPS --->");

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(gps.gpsTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do IMU (MPU9250)
// Inputs: nenhum
// Return: nenhum
void TaskIMU( void * pvParameters ) {
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task
    disableCore0WDT();              // Desabilita o watchdogtime de hardware do core 0

    //===========================================
    // Inicializacao do modulo IMU (MPU9250)
    Serial.println("Inicializando sensor IMU.....");
    if (!imu.begin()) {
        isActiveIMU = false;
    } else {
        isActiveIMU = true;
    }

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        // IMU
        if (isActiveIMU) {
            imu.read();
            if (imu.avaliable()) {

                strParametersIMU = imu.getStrDataIMU(now());

                if (sdCard.isSDCardPresent()) {
                    // sdCard.appendFile(SD, "/Teste.dat", esp32util.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU).c_str());
                    // String filename = "/Data_" + addressIdESP + "_" + year() + month() + day() + ".dat";
                    String filename = "/Data_IMU_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                    sdCard.appendFile(SD, filename.c_str(), esp32util.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU).c_str());
                    imu.numberOfRegisterIMU++;
                }

                if (isWorking_LoRa && imu.sendDataForLora) {
                    esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU));
                }
            } 
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(imu.IMUTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do IMU (MPU9250)
// Inputs: nenhum
// Return: nenhum
void Task5Min( void * pvParameters ) {
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (isWorking_LoRa) {
            String msg5min;
            msg5min = "[";
            msg5min += now();
            msg5min += ",";
            msg5min += String((unsigned long)imu.numberOfRegisterIMU);
            msg5min += ",";
            msg5min += esp32util.getVBat();
            msg5min += "]";
            // lora.send(msg5min);

        // esp_task_wdt_reset();                               // Reseta o contador do watchdog

            // Serial.println("Task5Min #1");

            // esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, String((unsigned long)imu.numberOfRegisterIMU)));
            esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, msg5min));

        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(esp32util.min5TaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
        // vTaskDelay(pdMS_TO_TICKS(10000));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}