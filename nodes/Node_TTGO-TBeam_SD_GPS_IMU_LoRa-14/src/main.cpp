#include "ESP32LibV2.h"
#include "esp_task.h"
#include "esp_task_wdt.h"

//===========================================
//  ESP32Lib
ESP32Lib esp32util = ESP32Lib();
String addressIdESP = esp32util.getMacAddress();

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-LoRa32-OLED-V1_SD_GPS_LoRa-13
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-LoRa32-OLED-V1_SD_GPS_LoRa-13")          // localizacao do projeto
#define MAIN_DEBUG                      (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)
#define MAIN_CORE_1                     (1)
#define MAIN_WATCHDOC_TIME_OUT          (15)            // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdLoRa;
TaskHandle_t TaskIdGPS;

void TaskSDCard( void * pvParameters );
void TaskLoRa( void * pvParameters );
void TaskGPS( void * pvParameters );

//===========================================
//  SD Card
#include "SDCardLibV3.h"

SDCardLib sdCard;

//===========================================
//  LoRa SX1276/8
#include "LoRaNodeLibV5.h"

LoRaNodeLib lora = LoRaNodeLib();
bool isWorking_LoRa = false;





void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

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

void loop() {
  // put your main code here, to run repeatedly:
}