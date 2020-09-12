#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\tests\Test_TTGO-TBeam_WiFi-OTA-03
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\tests\\Test_TTGO-TBeam_WiFi-OTA-03.00")          // localizacao do projeto
#define MAIN_DEBUG                      (true)          // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)             // Defini o nucleo a ser utilizado
#define MAIN_CORE_1                     (1)             // Defini o nucleo a ser utilizado
#define MAIN_WATCHDOC_TIME_OUT          (600)           // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdWiFiOTA;

void TaskWiFiOTA( void * pvParameters );

//===========================================
//  ESP32 Util
#include "ESP32NodeLibV20.h"

ESP32NodeLib module;
String addressIdESP = module.getIdModule();

boolean validDateTime = false;

//===========================================
//  Lib para implementar funcionalidade utilizando o modulo WiFi do ESP32
#include "WiFiOTALibV4.h"

WiFiOTALib wifi;

// boolean isConnectedOTA = false;
// boolean isConnectedNTP = false;
// boolean isConnectedWiFi = false;
// boolean isConnectedClientWebSocket = false;
// boolean isConnectedStationAP = false;
// byte activeConnections = B00000000;

boolean isConnectedSTA = false;                // Rede wifi conectada
boolean isConnectedAP = false;           // Um ou mais dispositivos conectado ao AP
boolean isConnectedOTA = false;                 // Servico OTA ativo
boolean isConnectedNTP = false;                 // Servico NTP ativo
boolean isConnectedClientWS = false;     // Um ou mais clientes conectados ao webserver
boolean isDateTimeValid = false;                // Data e hora validas

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    module.setUnixTime(0);
    Serial.print("Unix time: ");
    Serial.println(module.getUnixTimeNow());

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskWiFiOTA,     // Funcao com o codigo que implenta a Task
                    "TaskSDCard",   // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdWiFiOTA,  // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    // Habilitamos o watchdog com timeout de 15 segundos
    esp_task_wdt_init(MAIN_WATCHDOC_TIME_OUT, true);
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
        if ( (module.getUnixTimeNow() > INIT_UNIXTIME) && (module.getUnixTimeNow() < END_UNIXTIME) )
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
            }
        }

        // Imprime dados caso exista client conectado ao WebServer
        // Serial.println(__LINE__);
        // if ( (isConnectedSTA || isConnectedAP) && isConnectedClientWS )
        // {
        //     // Serial.println(__LINE__);
        //     wifi.printClientWS( String((unsigned long) module.getUnixTimeNow()) );
        // }

        wifi.printClientWS( String((unsigned long) module.getUnixTimeNow()) );

        // Nao precisa dessa parte, apenas para teste
        // Serial.println(__LINE__);
        Serial.print("module.getUnixTimeNow(): ");
        Serial.println( module.getUnixTimeNow() );

        if ( wifi.availableClientWS() > 0 )
        {
            Serial.println("WIFI_INFO: Client connected");
        }
        
        vTaskDelay(pdMS_TO_TICKS(WIFIOTA_TASK_DELAY_MS));       // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}