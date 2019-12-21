#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

                                        //D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\gateways\Gateway_TTGO-LoRa32-OLED-V1_THPA_MQTT_LoRa_MT-20
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\getways\\Gateway_TTGO-LoRa32-OLED-V1_THPA_MQTT_LoRa_MT-20")          // localizacao do projeto
#define MAIN_DEBUG                      (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (300)            // Tempo de espera do watchdog para reiniciar o processo (em segundos)

#define MQTT_TASK_MESSAGE               ("MQTT_TASK: ")
#define LORA_TASK_MESSAGE               ("LORA_TASK: ")
#define THPA_TASK_MESSAGE               ("THPA_TASK: ")

#define MQTT_TASK_TIME_SERIAL           (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define LORA_TASK_TIME_SERIAL           (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib

#define COMMAND_RESET_ESP               (99)

TaskHandle_t TaskIdTHPA;
TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdMQTT;

void TaskTHPA( void * pvParameters );
void TaskLoRa( void * pvParameters );
void TaskMQTT( void * pvParameters );

//===========================================
// MQTT
#include <MQTTLibV5.h>

MQTTLib mqttClient;
bool isWiFiConnected = false;
bool isMQTTConnected = false;

bool sendMQTT = false;
String strSendMQTT;
uint8_t numberOfReconnectAttempts = 10;
String addressIdESP = "2A3B4C";

#include <LinkedList.h>
LinkedList<String> listMessageSendMQTT = LinkedList<String>();
uint16_t numberMessageSendMQTT = 0;

//===========================================
// LoRa
#include "LoRaGatewayLibV6.h"

LoRaGatewayLib lora = LoRaGatewayLib();
bool isWorking_LoRa = false;
String recvLoRaString;

//===========================================
// THPA
#include "BME280LibV2.h"

BME280Lib thpa;

boolean isWorking_THPA = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 1
    xTaskCreatePinnedToCore(
                    TaskMQTT,       // Funcao com o codigo que implenta a Task
                    "TaskMQTT",     // Nome da Task
                    4096,          // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdMQTT,    // Referencia para acompanhar a Task
                    MAIN_CORE_1);        // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskLoRa,       // Funcao com o codigo que implenta a Task
                    "TaskLoRa",     // Nome da Task
                    4096,          // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdLoRa,    // Referencia para acompanhar a Task
                    MAIN_CORE_0);        // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskTHPA,       // Funcao com o codigo que implenta a Task
                    "TaskTHPA",     // Nome da Task
                    4096,          // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdTHPA,    // Referencia para acompanhar a Task
                    MAIN_CORE_0);        // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

    // Habilitamos o watchdog com timeout de 15 segundos
    esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
}

// Funcao que implementa a Task para o modulo do cliente MQTT
// Inputs: nenhum
// Return: nenhum
void TaskMQTT( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // MQTT
    isWiFiConnected = mqttClient.begin();
    isMQTTConnected = mqttClient.init();
    addressIdESP = mqttClient.getMacAddress();

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (mqttClient.mqttDebugTimeSerial) mqttClient.mqttDebugTimeSerial--;
        else {
            Serial.println(MQTT_TASK_MESSAGE);
            mqttClient.mqttDebugTimeSerial = MQTT_TASK_TIME_SERIAL;
        }

        if (mqttClient.checkWiFiConnection()) {

            uint8_t _cont = 10;

            while (numberMessageSendMQTT && listMessageSendMQTT.size() && _cont) {

                mqttClient.send(listMessageSendMQTT.remove(0));
                _cont--;
                esp_task_wdt_reset();               // Reseta o contador do watchdog
                vTaskDelay(pdMS_TO_TICKS(10));      // Suspende por 10 milisegundo o envio da proxima mensagem

            }

            mqttClient.loop();

            if (mqttClient.isNewRecvMsgMQTT()) {
                String strRecvMQTT = mqttClient.getReceivedMQTT();
                messageMQTT_t msgRecvMQTT = mqttClient.decoderStrJSonInObjectMsgMQTT(strRecvMQTT);
                if (msgRecvMQTT.id == addressIdESP) {
                    switch (msgRecvMQTT.type)
                    {
                    case COMMAND_RESET_ESP:
                        {
                            ESP.restart();
                        }
                        break;
                    
                    default:
                        break;
                    }

                    // if (msgRecvMQTT.type == COMMAND_RESET_ESP) {

                    // }

                } else {
                    lora.send(strRecvMQTT);
                }
            }

        } else {

            if (!mqttClient.connectWiFi() && (numberOfReconnectAttempts > 0)) {
                numberOfReconnectAttempts--;
            } else {
                esp_restart();
            }

            mqttClient.reconnect();

        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(mqttClient.mqttTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
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
        

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (lora.loraDebugTimeSerial) lora.loraDebugTimeSerial--;
        else {
            Serial.println(LORA_TASK_MESSAGE);
            lora.loraDebugTimeSerial = LORA_TASK_TIME_SERIAL;
        }

        if (lora.isReceived()) {
            if (lora.loraDebugMain) Serial.println("---> lora.isReceived()");

            String strRecv = lora.getStrRecv();
            messageLoRa_t messageRecv = lora.getMsgRecv();

            if (lora.loraDebugMain) Serial.print("strRecv: ");
            if (lora.loraDebugMain) Serial.println(strRecv);

            if (messageRecv.type != 9) {
            listMessageSendMQTT.add(strRecv);
            numberMessageSendMQTT++;
            } else {

                String commandEpochTime = "{\"id\":\"";
                commandEpochTime += messageRecv.id;
                commandEpochTime += "\",\"type\":10,\"payload\":[";
                commandEpochTime += (unsigned long)mqttClient.getEpochTime();
                commandEpochTime += "]}";

                lora.send(commandEpochTime);
            }

            if (lora.loraDebugMain) Serial.println("lora.isReceived() --->");
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog 
        vTaskDelay(pdMS_TO_TICKS(lora.loraTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do sensor temperatura, umidade, pressao barometrica e altitude do modulo BME280
// Inputs: nenhum
// Return: nenhum
void TaskTHPA( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // BME280
    isWorking_THPA = thpa.begin();;
    if (!isWorking_THPA)
        Serial.println("Error inicializacao do modulo BME280");
    else
        Serial.println("Modulo BME280 OK!");

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        Serial.println(THPA_TASK_MESSAGE);

        thpa.read();
        if (thpa.avaliable()) {

            String strTHPADatas = thpa.getStrDataTHPA(mqttClient.getEpochTime(), addressIdESP);
            listMessageSendMQTT.add(strTHPADatas);
            numberMessageSendMQTT++;

        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(thpa.thpaTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() {
}