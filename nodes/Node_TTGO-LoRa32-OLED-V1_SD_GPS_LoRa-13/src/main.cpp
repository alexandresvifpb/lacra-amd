// #include <Arduino.h>
#include "ESP32LibV2.h"
#include "esp_task.h"
#include "esp_task_wdt.h"

//===========================================
//  LoRa SX1276/8
// #include "ESP32LibV1.h"

ESP32Lib esp32util = ESP32Lib();
String addressIdESP = esp32util.getMacAddress();


// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-LoRa32-OLED-V1_SD_GPS_LoRa-13
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-LoRa32-OLED-V1_SD_GPS_LoRa-13")          // localizacao do projeto
#define MAIN_DEBUG                      (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (15)            // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdSDCard;

void TaskLoRa( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskSDCard( void * pvParameters );

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV4.h"

LoRaNodeLib lora = LoRaNodeLib();
bool isWorking_LoRa = false;
// String recvLoRaString;

// LinkedList<String> listMessageSendLoRa = LinkedList<String>();
// uint16_t numberMessageSendLoRa = 0;

//===========================================
//  GPS NEO6MV2
#include "GPSLibV5.h"

GPSLib gps;

boolean isWorking_GPS = false;
// String strParametersGPS;

//===========================================
//  SD Card
#include "SDCardLibV2.h"

SDCardLib sdCard;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    // esp32util.begin();

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 1
    xTaskCreatePinnedToCore(
                    TaskLoRa,       // Funcao com o codigo que implenta a Task
                    "TaskLoRa",     // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdLoRa,    // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskGPS,        // Funcao com o codigo que implenta a Task
                    "TaskGPS",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdGPS,     // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskSDCard,     // Funcao com o codigo que implenta a Task
                    "TaskSDCard",   // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdSDCard,  // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    // Habilitamos o watchdog com timeout de 15 segundos
    esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
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
        
    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (loraTimeLoopSerial) loraTimeLoopSerial--;
        else {
            Serial.println(LORA_DEBUG_MESSAGE);
            loraTimeLoopSerial = lora.loraDebugTimeSerial;
        }

        // if (lora.loraDebugSend && esp32util.avaliableSendMessage()) Serial.println("lora.loraDebugSend: true");
        // if (esp32util.avaliableSendMessage()) Serial.println("esp32util.avaliableSendMessage(): true");

        uint8_t _cont = 10;

        while (esp32util.avaliableSendMessage() && _cont) {

            String strMessageSend = esp32util.getNextSendMessage();

            if (lora.loraDebugSend) Serial.println("---> LoraSend");
            if (lora.loraDebugSend) Serial.println(strMessageSend);

            lora.send(strMessageSend);
            _cont--;
            if (sdCard.isSDCardPresent()) {
                sdCard.appendFile(SD, "/Teste.dat", strMessageSend.c_str());
            }

        }

/*
        if (lora.isReceived()) {
            if (lora.loraDebugMain) Serial.println("---> lora.isReceived()");

            String strRecv = lora.getStrRecv();

            if (lora.loraDebugMain) Serial.print("strRecv: ");
            if (lora.loraDebugMain) Serial.println(strRecv);

            listMessageSendLoRa.add(strRecv);
            numberMessageSendLoRa++;

            if (lora.loraDebugMain) Serial.println("lora.isReceived() --->");
        }
*/
        esp_task_wdt_reset();                               // Reseta o contador do watchdog 
        vTaskDelay(pdMS_TO_TICKS(lora.loraTaskDelayMS));    // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para o modulo do cliente MQTT
// Inputs: nenhum
// Return: nenhum
void TaskSDCard( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // SD Card
    if (!sdCard.begin())
        Serial.println("Error inicializacao do modulo SDcard");
    else
        Serial.println("SDCard encontrado!");

    uint16_t sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (sdCardTimeLoopSerial) sdCardTimeLoopSerial--;
        else {
            Serial.println(SD_DEBUG_MESSAGE);
            sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        // vTaskDelay(pdMS_TO_TICKS(MQTT_TASK_DELAY_MS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
        vTaskDelay(pdMS_TO_TICKS(sdCard.sdTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do sensor temperatura, umidade, pressao barometrica e altitude do modulo BME280
// Inputs: nenhum
// Return: nenhum
void TaskGPS( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // Inicializacao do modulo GPS
    isWorking_GPS = gps.begin();;
    if (isWorking_GPS)
        gps.requestSync();

    uint16_t gpsTimeLoopSerial = gps.gpsDebugTimeSerial;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (gpsTimeLoopSerial) gpsTimeLoopSerial--;
        else {
            Serial.println(GPS_DEBUG_MESSAGE);
            gpsTimeLoopSerial = gps.gpsDebugTimeSerial;
        }

        if (gps.gpsDebugGetData) Serial.println("---> GPS");

        if (gps.read() && gps.gpsDebugGetData) Serial.println("---> GPS Read");

        if (gps.avaliable()) {
            if (gps.gpsDebugGetData) Serial.println("---> GPS avaliable");
            String strDataGPS = gps.getDataString(gps.getEpochTime());
            if (gps.gpsDebugGetData) Serial.println(strDataGPS);
            esp32util.addNewSendMessage(esp32util.encoderJSon(addressIdESP, GPS_TYPE, strDataGPS));
        }

        if (gps.gpsDebugGetData) Serial.println(gps.gpsTaskDelayMS);
        if (gps.gpsDebugGetData) Serial.println("GPS --->");

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(gps.gpsTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() {
}