// #include <Arduino.h>
#include "ESP32NodeLibV18.h"
#include "esp_task.h"
#include "esp_task_wdt.h"

//===========================================
//  ESP32 Util

// ESP32NodeLib esp32util = ESP32NodeLib();
ESP32NodeLib esp32util;
String addressIdESP = esp32util.getMacAddress();
boolean connectionOTA = false;

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-TBeam_SD_GPS_IMU_LoRa_OTA-18
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-TBeam_SD_GPS_IMU_LoRa_OTA-18.05")          // localizacao do projeto
#define MAIN_DEBUG                      (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (600)            // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdIMU;
TaskHandle_t TaskIdTime;

void TaskSDCard( void * pvParameters );
void TaskLoRa( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskIMU( void * pvParameters );
void TaskTime( void * pvParameters );

//===========================================
//  SD Card
#include "SDCardLibV17.h"

SDCardLib sdCard;
boolean isActiveSDCard = false;

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV17.h"

// LoRaNodeLib lora = LoRaNodeLib();
LoRaNodeLib lora;
boolean isActiveLoRa = false;

//===========================================
//  GPS NEO6MV2
#include "GPSLibV18.h"

GPSLib gps;
boolean isActiveGPS = false;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMULibV17.h"

// IMULib imu = IMULib();
IMULib imu;
String strParametersIMU;
boolean isActiveIMU = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    connectionOTA = esp32util.begin();

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
                    TaskTime,        // Funcao com o codigo que implenta a Task
                    "TaskTime",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdTime,     // Referencia para acompanhar a Task
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
    isActiveSDCard = sdCard.begin();
    if (!isActiveSDCard)
        Serial.println("Error inicializacao do modulo SDcard");
    else
        Serial.println("SDCard encontrado!");

    uint16_t sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (isActiveSDCard) {
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
    isActiveLoRa = lora.begin();;
    if (!isActiveLoRa)
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

            // if (lora.loraDebugSend) Serial.println("---> LoraSend");
            // if (lora.loraDebugSend) Serial.println(strMessageSend);

            // Serial.print("LORA_SEND: ");
            // Serial.println(strMessageSend);

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
                    esp32util.timeTaskDelayMS = atol(strPayload.c_str());
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
    isActiveGPS = gps.begin();;
    if (isActiveGPS)
        gps.requestSync();

    uint16_t gpsTimeLoopSerial = gps.gpsDebugTimeSerial;
    // uint16_t counterSendGPS = gps.tsSendIntervalGPS;
    uint64_t lastSendGPS = millis();

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        esp_task_wdt_reset();                               // Reseta o contador do watchdog

        Serial.println(GPS_DEBUG_MESSAGE);

        if (gpsTimeLoopSerial) gpsTimeLoopSerial--;
        else {
            Serial.println(GPS_DEBUG_MESSAGE);
            gpsTimeLoopSerial = gps.gpsDebugTimeSerial;
        }

        if (gps.gpsDebugGetData) Serial.println("---> GPS");

        if (gps.read() && gps.gpsDebugGetData) Serial.println("---> GPS Read");

        // if (gps.avaliable() && counterSendGPS) {
        if (gps.avaliable()) {
            if (gps.gpsDebugGetData) Serial.println("---> GPS avaliable");
            // String strDataGPS = gps.getDataString(gps.getEpochTime());
            // String strDataGPS = gps.getDataString(esp32util.getEpochTime());
            
            String strDataGPS = gps.getDataString(now());
            if (gps.gpsDebugGetData) Serial.println(strDataGPS);
            // esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS));

            // if (!counterSendGPS) {
            if ( (millis() - lastSendGPS) >= gps.tsSendIntervalGPS ) {

                Serial.println("lastSendGPS loop");

                esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS));

                if (sdCard.isSDCardPresent()) {
                    String filename = "/Data_GPS_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                    sdCard.appendFile(SD, filename.c_str(), esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS).c_str());
                }
                // counterSendGPS = gps.tsSendIntervalGPS;
                // counterSendGPS--;
                lastSendGPS = millis();
            } else if (sdCard.isSDCardPresent() && gps.checkMovement()) {
                String filename = "/Data_GPS_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                sdCard.appendFile(SD, filename.c_str(), esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS).c_str());
                // counterSendGPS = gps.tsSendIntervalGPS;
            }

            // counterSendGPS = gps.tsSendIntervalGPS;
            // counterSendGPS--;

            // Serial.print("counterSendGPS: ");
            // Serial.println(counterSendGPS);

            Serial.println("GPS loop");
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

                if (isActiveLoRa && imu.sendDataForLora) {
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
void TaskTime( void * pvParameters ) {
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (isActiveLoRa) {
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

            // Serial.println("TaskTime #1");

            // esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, String((unsigned long)imu.numberOfRegisterIMU)));
            esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, msg5min));

        }

        // if (connectionOTA) {
        //     //Handle é descritor que referencia variáveis no bloco de memória
        //     //Ele é usado como um "guia" para que o ESP possa se comunicar com o computador pela rede
        //     ArduinoOTA.handle();
        // } else         {
        //     connectionOTA = esp32util.connectOTA();
        // }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(esp32util.timeTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
        // vTaskDelay(pdMS_TO_TICKS(10000));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
    }
}

void loop() {
  // put your main code here, to run repeatedly:

        if (connectionOTA) {
            //Handle é descritor que referencia variáveis no bloco de memória
            //Ele é usado como um "guia" para que o ESP possa se comunicar com o computador pela rede
            ArduinoOTA.handle();

            // Serial.println("loop");

        } else         {
            connectionOTA = esp32util.connectOTA();
        }
}