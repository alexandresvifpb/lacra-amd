#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"
                                        //D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\gateways\Gateway_TTGO-LoRa32-OLED-V1_THPA_MQTT_LoRa_MT-20
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\getways\\Gateway_TTGO-LoRa32-OLED-V1_MQTT_LoRa_Movel-Oled_WiFi-OTA-22")          // localizacao do projeto
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

TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdWiFiOTA;
TaskHandle_t TaskIdOLED;

void TaskLoRa( void * pvParameters );
void TaskWiFiOTA( void * pvParameters );
void TaskOLED( void * pvParameters );

//===========================================
//  ESP32 Util
#include "EPS32GatewayLibV1.h"

ESP32GatewayLib module;
String addressIdESP = module.getIdModule();

boolean validDateTime = false;
configs_t configs;
uint16_t bootseq;

//===========================================
// LoRa
#include "LoRaGatewayLibV7.h"

LoRaGatewayLib lora = LoRaGatewayLib();
boolean isWorkingLoRa = false;
String recvLoRaString;

//===========================================
//  Lib para implementar funcionalidade utilizando o modulo WiFi do ESP32
#include "WiFiGatewayOTALibV2.h"

WiFiGatewayOTALib wifi;

boolean isConnectedSTA = false;                // Rede wifi conectada
boolean isConnectedAP = false;           // Um ou mais dispositivos conectado ao AP
boolean isConnectedOTA = false;                 // Servico OTA ativo
boolean isConnectedNTP = false;                 // Servico NTP ativo
boolean isConnectedClientWS = false;     // Um ou mais clientes conectados ao webserver
boolean isDateTimeValid = false;                // Data e hora validas

//===========================================
// LoRa
#include "OLEDLibV1.h"

OLEDLib oled = OLEDLib();

boolean isWorkingOLED = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    EEPROM.begin(2);
    // bootseq = module.getBootSequence();
    configs.bootSequence = module.getBootSequence();
    Serial.print("Boot Sequence: ");
    // Serial.println(bootseq);
    Serial.println(configs.bootSequence);

    configs.id = addressIdESP;
    configs.seconds = 0;           // contador de segundos desde a inicializacao do dispositivo

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskLoRa,       // Funcao com o codigo que implenta a Task
                    "TaskLoRa",     // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdLoRa,    // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskWiFiOTA,    // Funcao com o codigo que implenta a Task
                    "TaskWiFiOTA",  // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdWiFiOTA, // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskOLED,       // Funcao com o codigo que implenta a Task
                    "TaskOLED",     // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdOLED,    // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(200);                     // delay para proximo comando

}

// Funcao que implementa a Task para o modulo de radio LoRa
// Inputs: nenhum
// Return: nenhum
void TaskLoRa( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // LoRa
    isWorkingLoRa = lora.begin();;
    if (!isWorkingLoRa)
        Serial.println("Error inicializacao do modulo LoRa");
    else
        Serial.println("Modulo LoRa OK!");
        
    int cont = 0;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        if (lora.loraDebugTimeSerial) lora.loraDebugTimeSerial--;
        else {
            Serial.println(LORA_TASK_MESSAGE);
            lora.loraDebugTimeSerial = LORA_TASK_TIME_SERIAL;
            oled.setLoRaMessageShowOledScreen(String(cont));
            cont++;
        }

        if (lora.isReceived()) {
            if (lora.loraDebugMain) Serial.println("---> lora.isReceived()");

            String strRecv = lora.getStrRecv();
            messageLoRa_t messageRecv = lora.getMsgRecv();

            if (lora.loraDebugMain) Serial.print("strRecv: ");
            if (lora.loraDebugMain) Serial.println(strRecv);

            oled.setOTAMessageShowOledScreen(messageRecv.payload);

/*
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
*/

            if (lora.loraDebugMain) Serial.println("lora.isReceived() --->");
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog 
        vTaskDelay(pdMS_TO_TICKS(lora.loraTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para o WiFi
// Inputs: nenhum
// Return: nenhum
void TaskWiFiOTA( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Inicializa o modulo wifi do ESP32
    wifi.begin();

    // Loop infinito obrigatorio para manter a Task rodando
    while(true)
    {
        esp_task_wdt_reset();                                   // Reseta o contador do watchdog

        isConnectedSTA = wifi.isConnectedSTA();
        isConnectedAP = wifi.isConnectedAP();

        // Serial.println(__LINE__);
        if ( !isConnectedSTA && !isConnectedAP )
        {
            // Serial.println(__LINE__);
            if ( wifi.initSTA() )
            {
                // Serial.println(__LINE__);
                if ( wifi.isConnectedSTA() )
                {
                    isConnectedOTA = wifi.activeOTA();
                    isConnectedNTP = wifi.bootServiceNTP();
                    isConnectedSTA = true;
                }
            }
            else
            {
                // Serial.println(__LINE__);
                isConnectedAP = wifi.initAP();
            }
        }
        
        if ( isConnectedOTA )
        {
            // Handle é descritor que referencia variáveis no bloco de memória
            // Ele é usado como um "guia" para que o ESP possa se comunicar com o computador pela rede
            // Serial.println(__LINE__);
            ArduinoOTA.handle();

        }
        else if ( isConnectedSTA )
        {
            // Serial.println(__LINE__);
            isConnectedOTA = wifi.activeOTA();
        }

        // verifica se data e horas sao validos
        // Serial.println(__LINE__);
        if ( !module.isValidUnixTime(module.getUnixTimeNow()) )
        {
            // Serial.println(__LINE__);
            isDateTimeValid = true;
        }
        else
        {
            // Serial.println(__LINE__);
            isDateTimeValid = false;
        }

        // Corrige a data hora caso esteja invalidos        
        // Serial.println(__LINE__);
        if ( isConnectedNTP && !isDateTimeValid )
        {
            // Serial.println(__LINE__);
            uint64_t _unixTime = wifi.getUnixTimeNTP();
            module.setUnixTime(_unixTime);
        }

        // Verifica se ocorreu alguma conexao de client WebServer
        // Serial.println(__LINE__);
        if ( (isConnectedSTA || isConnectedAP) )
        {
            // Serial.println(__LINE__);
            if ( wifi.listenForClientsWS() == 1)
            {
                // Serial.println(__LINE__);
                isConnectedClientWS = true;
            }

            if ( wifi.availableClientWS() == 0)
            {
                // Serial.println(__LINE__);
                isConnectedClientWS = false;
            }
            
        }

        // Verifica se existe dados chengando pelo WebServer            
        // Serial.println(__LINE__);
        if ( isConnectedClientWS )
        {
            // Serial.println(__LINE__);
            if ( wifi.availableRecvDataClientWS() )
            {
                // Serial.println(__LINE__);
                String recvCommandWebSocket = wifi.readClientWS();

                Serial.print("RECV WEB SOCKET: ");
                Serial.println(recvCommandWebSocket);

                commands_t command = module.getCommand(recvCommandWebSocket);

                if ( command.type == 1 )
                {
                    module.setUnixTime(atol(command.payload.c_str()));
                }
            }
        }

        if ( wifi.availableClientWS() > 0 )
        {
            Serial.println("WIFI_INFO: Client connected");
        }

        vTaskDelay(pdMS_TO_TICKS(tsWiFiOTATaskDelayMS));       // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do sensor temperatura, umidade, pressao barometrica e altitude do modulo BME280
// Inputs: nenhum
// Return: nenhum
void TaskOLED( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    //===========================================
    // OLED
    isWorkingOLED = oled.begin();
    if (!isWorkingOLED)
        Serial.println("Error inicializacao do modulo OLED");
    else
        Serial.println("Modulo OLED OK!");

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) {

        oled.run();

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(oled.tsOledTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}