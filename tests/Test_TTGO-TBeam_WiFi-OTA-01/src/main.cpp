#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\tests\Test_TTGO-TBeam_WiFi-OTA-01
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\tests\\Test_TTGO-TBeam_WiFi-OTA-01.04")          // localizacao do projeto
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
#include "WiFiOTALibV2.h"

// #define BLEWIFIOTA_TASK_DELAY_MS           (1000)

WiFiOTALib wifi;

boolean isConnectedOTA = false;
boolean isConnectedNTP = false;
boolean isConnectedWiFi = false;
boolean isConnectedClientWebSocket = false;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
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

// Funcao que implementa a Task para o SDCard
// Inputs: nenhum
// Return: nenhum
void TaskWiFiOTA( void * pvParameters ) {
    esp_task_wdt_add(NULL);

    // Inicializa o modulo wifi do ESP32
    wifi.begin();
    // deviceBLE.begin();

    // Verifica se a conexao wifi esta ativa e salva na variavel
    isConnectedWiFi = wifi.isConnectedWiFi();

    // Se a rede wifi estiver ativa inicializa os servicos
    if ( isConnectedWiFi )
    {
        // inicializa o servico OTA
        isConnectedOTA = wifi.activeOTA();

        // inicializa o servico NTP
        isConnectedNTP = wifi.bootServiceNTP();
    } 

    // Loop infinito obrigatorio para manter a Task rodando
    while(true)
    {
        esp_task_wdt_reset();                                   // Reseta o contador do watchdog

        // Verifica se a rede wifi esta contectada
        // TRUE: seta a variavel
        // FALSE: faz um nova tentativa de reconexao
        if ( wifi.isConnectedWiFi() )
        {
            isConnectedWiFi = true;
        }
        else
        {
            // Verifica se a tentativa de reconexao foi bem sucedida
            // TRUE: ativa os serviço de OTA e NTP
            // FALSE: seta variaveis
            if ( wifi.reconnectWiFi() )
            {
                // inicializa o servico OTA
                isConnectedOTA = wifi.activeOTA();

                // inicializa o servico NTP
                isConnectedNTP = wifi.bootServiceNTP();
            }
            else
            {
                isConnectedWiFi = false;
                isConnectedOTA = false;
                isConnectedNTP = false;
            }
        }
        
        // Verifica se o servico OTA esta ativo
        // TRUE: realiza uma verificacao de uma nova tentativa de conexao
        // FALSE: verifica se a rede wifi esta ativa
        if ( isConnectedOTA ) 
        {
            // Handle é descritor que referencia variáveis no bloco de memória
            // Ele é usado como um "guia" para que o ESP possa se comunicar com o computador pela rede
            ArduinoOTA.handle();
        } 
        else if ( isConnectedWiFi )
        {
            isConnectedOTA = wifi.activeOTA();
        }   

        // Verifica se o UnixTime é valido
        // TRUE: seta variave de validade da data e time
        if ( (module.getUnixTimeNow() > INIT_UNIXTIME) && (module.getUnixTimeNow() < END_UNIXTIME) )
        {
            validDateTime = true;
        }

        // Verifica se o serviço NTP esta ativo e se o UnixTime é invalido
        // TRUE: procedimento para atualizar o UnixTime utilizando o serivico NTP
        if ( isConnectedNTP && !validDateTime )
        {
            uint64_t _unixTime = wifi.getUnixTimeNTP();
            module.setUnixTime(_unixTime);
                
            Serial.print("UnixTimeNTP: ");
            Serial.println( (unsigned long) _unixTime );

            wifi.printWebSocket( String((unsigned long) module.getUnixTimeNow()) );
        }

        if ( isConnectedWiFi && (wifi.listenForClientsSocket() == 1) )
        {
            isConnectedClientWebSocket = true;
        }
            
        if ( isConnectedClientWebSocket )
        {
            if ( wifi.availableRecvData() )
            {
                String recvCommandWebSocket = wifi.readWebSocket();

                Serial.print("RECV WEB SOCKET: ");
                Serial.println(recvCommandWebSocket);
            }

            if ( !wifi.availableConnection() )
            {
                isConnectedClientWebSocket = false;
            }
        }

        if ( isConnectedClientWebSocket )
        {
            wifi.printWebSocket( String((unsigned long) module.getUnixTimeNow()) );
            // deviceBLE.print( String((unsigned long) module.getUnixTimeNow()) );
        }

        // Nao precisa dessa parte, apenas para teste
        Serial.print("module.getUnixTimeNow(): ");
        Serial.println( module.getUnixTimeNow() );

        vTaskDelay(pdMS_TO_TICKS(WIFIOTA_TASK_DELAY_MS));       // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() {
  // put your main code here, to run repeatedly:
}