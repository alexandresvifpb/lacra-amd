#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

//===========================================
//  ESP32 Util
#include "ESP32NodeLibV19.h"

ESP32NodeLib esp32util;
String addressIdESP = esp32util.getMacAddress();
Config_t config;                                        // vai ser usado para configurar as variaveis de intervalo das task usando um arquivo no sdcard ou na memoria flash
boolean connectionOTA = false;
allDatas_t allDatas;
boolean validDateTime = false;                          // indica que o epochtime utilizado nos registro foi validado por acesso ao servidor NTP ou pelo GPS

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-TBeam_SD_GPS_IMU-DMP_LoRa_OTA-19
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-TBeam_SD_GPS_IMU-DMP_LoRa_OTA-19.04")          // localizacao do projeto
#define MAIN_DEBUG                      (true)          // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (600)           // Tempo de espera do watchdog para reiniciar o processo

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
#include "SDCardLibV19.h"

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
#include "GPSLibV19.h"

GPSLib gps;
boolean isActiveGPS = false;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMULibV19.h"

IMULib sensorIMU;
boolean isActiveIMU = false;
String strParametersIMU;
int numberOfRegisterIMU;
boolean sendDataForLoraIMU = false;
EulerAngles_t lastDatasEulerAngles, _datasEulerAngles;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    connectionOTA = esp32util.begin();

    if (now() > GPS_EPOCH_TIME_2019)
    {
        validDateTime = true;
    }

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
    if (!isActiveSDCard) {
        Serial.println("Error inicializacao do modulo SDcard");
    }
    else
    {
        Serial.println("SDCard encontrado!");
    }

    uint16_t sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;
    allDatas.id = addressIdESP;

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
        // vTaskDelay(pdMS_TO_TICKS(config.sdTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
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

                    if ( now() > GPS_EPOCH_TIME_2019 )
                    {
                        validDateTime = true;
                    }
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
                    // if (atol(strPayload.c_str())) imu.sendDataForLora = true;
                    // else imu.sendDataForLora = false;
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
        // vTaskDelay(pdMS_TO_TICKS(config.loraTaskDelayMS));    // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do GPS
// Inputs: void
// Return: void
void TaskGPS( void * pvParameters ) {
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task

    //===========================================
    // Inicializa o modulo GPS e retorna TRUE se bem sucedido
    isActiveGPS = gps.begin();

    // Loop infinito obrigatorio para manter a Task rodando
    while( true ) {

        esp_task_wdt_reset();                               // Reseta o contador do watchdog

        // Verifica se o GPS esta ativo
        if ( isActiveGPS )
        {
            // Verifica se ha novos dados no GPS
            if ( gps.avaliable() )
            {
                gps.update();   // recupera os novos dados do gps e salva em currentPosition

                // Sincroniza data e hora utilizando dados do GPS caso ainda não tenha sido sincronizado
                // utilizando o servidor NTP por meio da conexao WiFi
                if ( !validDateTime )
                {
                    if (gps.requestSync())
                    {
                        validDateTime = true;
                    }
                } 

                // Atualiza allDatas com a novo localizacao
                allDatas.epochTimeGPS = now();
                allDatas.latitude = gps.currentPosition.latitude;
                allDatas.longitude = gps.currentPosition.longitude;
                allDatas.altitude = gps.currentPosition.altitude;
                allDatas.speed = gps.currentPosition.speed;

                // Testes para verificar se a necessidade de enviar e salvar uma nova posicao.
                // Teste 1: Calcula a distancia entre a atual posicao e relacao a anterior e verifica se
                // essa distancia é superior ao error de acuracia do GPS.
                // Teste 2: Verifica se o intervalo de tempo entre cada envio de dados foi atingido.
                if ( (gps.calculateDelta(gps.lastPosition, gps.currentPosition) >= GPS_POSITION_ACCURACY/2) || ( gps.checkTimeInterval() ) )
                {
                    String currentPosition = gps.getCurretPosition(allDatas.epochTimeGPS);       // recupera uma string da atual posicao 
                    
                    // Adiciona uma nova localizacao na lista a ser enviada pelo modulo LoRa
                    esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, GPS_TYPE, currentPosition)); 

                    // Verifica se um sdCard esta presente
                    if ( isActiveSDCard )
                    {
                        // Adiciona um novo registro no arquivo "Data_GPS"
                        String filename = "/Data_GPS_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                        sdCard.appendFile(SD, filename.c_str(), esp32util.encoderJSon(addressIdESP, GPS_TYPE, currentPosition).c_str());

                        // Adiciona um novo registro no arquivo "Data_ALL"
                        filename = "/Data_ALL_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                        sdCard.appendFile(SD, filename.c_str(), esp32util.getAllDatas(allDatas).c_str());

                    }
                }
            }
        }

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
    if (!sensorIMU.begin()) {
        isActiveIMU = false;
    } else {
        isActiveIMU = true;
    }

    // EulerAngles_t lastDatasEulerAngles;
    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        // IMU
        if (isActiveIMU) {
            
            if (sensorIMU.avaliable()) {

                if (sensorIMU.updateDatas()) {

                    IMU_t _datasIMU = sensorIMU.getIMU();
                    Quaternion_t _datasQuaternion = sensorIMU.getQuaternion();
                    // EulerAngles_t _datasEulerAngles = sensorIMU.getEulerAngles();
                    _datasEulerAngles = sensorIMU.getEulerAngles();

                    time_t _epochTimeNow = now();
                    allDatas.dateTime = String(month(_epochTimeNow)) + "/" + String(day(_epochTimeNow)) + "/" + String(year(_epochTimeNow)) + " " + String(hour(_epochTimeNow)) + ":" + String(minute(_epochTimeNow)) + ":" + String(second(_epochTimeNow));
                    allDatas.epochTimeIMU = _epochTimeNow;
                    allDatas.accelX = _datasIMU.accelX;
                    allDatas.accelY = _datasIMU.accelY;
                    allDatas.accelZ = _datasIMU.accelZ;
                    allDatas.gyroX = _datasIMU.gyroX;
                    allDatas.gyroY = _datasIMU.gyroY;
                    allDatas.gyroZ = _datasIMU.gyroZ;
                    allDatas.magX = _datasIMU.magX;
                    allDatas.magY = _datasIMU.magY;
                    allDatas.magZ = _datasIMU.magZ;
                    allDatas.q0 = _datasQuaternion.q0;
                    allDatas.q1 = _datasQuaternion.q1;
                    allDatas.q2 = _datasQuaternion.q2;
                    allDatas.q3 = _datasQuaternion.q3;
                    allDatas.Yam = _datasEulerAngles.Yam;
                    allDatas.Pitch = _datasEulerAngles.Pitch;
                    allDatas.Roll = _datasEulerAngles.Roll;

                    // strParametersIMU = sensorIMU.getStrDatas(now(), _datasIMU, _datasQuaternion, _datasEulerAngles);
                    strParametersIMU = sensorIMU.getStrDatas(now(), _datasIMU, _datasQuaternion, _datasEulerAngles);

                    boolean _newData = false;
                    if ((_datasEulerAngles.Yam - lastDatasEulerAngles.Yam) > DATA_YAM_DIF ||
                        (_datasEulerAngles.Pitch - lastDatasEulerAngles.Pitch) > DATA_PITCH_DIF ||
                        (_datasEulerAngles.Roll - lastDatasEulerAngles.Roll) > DATA_ROLL_DIF) {
                            _newData = true;
                        }
                    lastDatasEulerAngles = _datasEulerAngles;

                    if (sdCard.isSDCardPresent() && _newData) {
                        String filename = "/Data_ALL_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                        sdCard.appendFile(SD, filename.c_str(), esp32util.getAllDatas(allDatas).c_str());
                        numberOfRegisterIMU++;
                    }

                    if (sdCard.isSDCardPresent() && _newData) {
                        String filename = "/Data_IMU_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                        sdCard.appendFile(SD, filename.c_str(), esp32util.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU).c_str());
                        numberOfRegisterIMU++;
                    }

                    if (isActiveLoRa && sendDataForLoraIMU) {
                        esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU));
                    }
                }
            }
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(sensorIMU.imuTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
        // vTaskDelay(pdMS_TO_TICKS(config.imuTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
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
            msg5min += String((unsigned long)numberOfRegisterIMU);
            msg5min += ",";
            msg5min += esp32util.getVBat();
            msg5min += "]";
            // lora.send(msg5min);

        // esp_task_wdt_reset();                               // Reseta o contador do watchdog

            // Serial.println("TaskTime #1");

            // esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, String((unsigned long)imu.numberOfRegisterIMU)));
            // esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, 7, String((unsigned long)numberOfRegisterIMU)));
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
        // vTaskDelay(pdMS_TO_TICKS(120000));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
        vTaskDelay(pdMS_TO_TICKS(esp32util.timeTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
        // vTaskDelay(pdMS_TO_TICKS(config.timeTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade (milisegundos)
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