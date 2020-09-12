#include <Arduino.h>
#include "esp_task.h"
#include "esp_task_wdt.h"

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\nodes\Node_TTGO-TBeam_GPS_IMU-DMP_SD_LoRa_WiFi-OTA-WS-21
#define MAIN_MESSAGE_INITIAL            ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\nodes\\Node_TTGO-TBeam_GPS_IMU-DMP_SD_WiFi-OTA-WS-22.01")          // localizacao do projeto
#define MAIN_DEBUG                      (true)          // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa
#define MAIN_CORE_0                     (0)             // Defini o nucleo a ser utilizado
#define MAIN_CORE_1                     (1)             // Defini o nucleo a ser utilizado
#define MAIN_WATCHDOC_TIME_OUT          (600)           // Tempo de espera do watchdog para reiniciar o processo

TaskHandle_t TaskIdWiFiOTA;
TaskHandle_t TaskIdSDCard;
TaskHandle_t TaskIdGPS;
TaskHandle_t TaskIdIMU;

void TaskWiFiOTA( void * pvParameters );
void TaskSDCard( void * pvParameters );
void TaskGPS( void * pvParameters );
void TaskIMU( void * pvParameters );

//===========================================
//  ESP32 Util
#include "ESP32NodeLibV21.h"

ESP32NodeLib module;
String addressIdESP = module.getIdModule();

boolean validDateTime = false;
allDatas_t allDatas;
uint16_t bootseq;
uint16_t conterALL = 0;
uint16_t conterGPS = 0;
uint16_t conterIMU = 0;

//===========================================
//  Lib para implementar funcionalidade utilizando o modulo WiFi do ESP32
#include "WiFiOTALibV4.h"

WiFiOTALib wifi;

boolean isConnectedSTA = false;                // Rede wifi conectada
boolean isConnectedAP = false;           // Um ou mais dispositivos conectado ao AP
boolean isConnectedOTA = false;                 // Servico OTA ativo
boolean isConnectedNTP = false;                 // Servico NTP ativo
boolean isConnectedClientWS = false;     // Um ou mais clientes conectados ao webserver
boolean isDateTimeValid = false;                // Data e hora validas

//===========================================
//  SD Card
#include "SDCardLibV19.h"

SDCardLib sdCard;
boolean isActiveSDCard = false;
int gravar = 1000;

//===========================================
//  GPS NEO6MV2
#include "GPSLibV20.h"

GPSLib gps;
boolean isActiveGPS = false;

//===========================================
//  Accelerometer & Gyroscope & Magnetometer MPU9255 (GY-91) 10DOF
#include "IMULibV19.h"

IMULib sensorIMU;
boolean isActiveIMU = false;
String strParametersIMU;
int numberOfRegisterIMU;
// boolean sendDataForLoraIMU = false;
EulerAngles_t lastDatasEulerAngles, _datasEulerAngles;
uint64_t currentTimeSendLora = millis();

void setup() 
{
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MAIN_MESSAGE_INITIAL);
    Serial.println();

    module.setUnixTime(0);
    Serial.print("Unix time: ");
    Serial.println(module.getUnixTimeNow());

    EEPROM.begin(2);
    // bootseq = module.getBootSequence();
    allDatas.bootSequence = module.getBootSequence();
    Serial.print("Boot Sequence: ");
    // Serial.println(bootseq);
    Serial.println(allDatas.bootSequence);

    allDatas.id = addressIdESP;
    allDatas.seconds = 0;           // contador de segundos desde a inicializacao do dispositivo

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskWiFiOTA,     // Funcao com o codigo que implenta a Task
                    "TaskSDCard",   // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    1,              // Prioridade da Task
                    &TaskIdWiFiOTA,  // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskTHPA(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskSDCard,     // Funcao com o codigo que implenta a Task
                    "TaskSDCard",   // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    2,              // Prioridade da Task
                    &TaskIdSDCard,  // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskGPS,        // Funcao com o codigo que implenta a Task
                    "TaskGPS",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    3,              // Prioridade da Task
                    &TaskIdGPS,     // Referencia para acompanhar a Task
                    MAIN_CORE_1);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
    delay(100);                     // delay para proximo comando

    //Cria uma Task que executará a funcao TaskLoRa(), com prioridade 1 e rodando no nucleo 0
    xTaskCreatePinnedToCore(
                    TaskIMU,        // Funcao com o codigo que implenta a Task
                    "TaskIMU",      // Nome da Task
                    4096,           // Tamanho da pilha (stack) a serem alocada na criacao da Task
                    NULL,           // Parametros de entrada da Task
                    3,              // Prioridade da Task
                    &TaskIdIMU,     // Referencia para acompanhar a Task
                    MAIN_CORE_0);   // Nucleo no qual a Task rodara (0 ou 1 para o ESP32)
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
            // uint64_t _unixTime = wifi.getUnixTimeNTP();
            // module.setUnixTime(_unixTime);
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

        // vTaskDelay(pdMS_TO_TICKS(WIFIOTA_TASK_DELAY_MS));       // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
        vTaskDelay(pdMS_TO_TICKS(tsWiFiOTATaskDelayMS));       // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para o SDCard
// Inputs: nenhum
// Return: nenhum
void TaskSDCard( void * pvParameters ) 
{
    esp_task_wdt_add(NULL);

    //===========================================
    // SD Card
    isActiveSDCard = sdCard.begin();
    if (!isActiveSDCard) 
    {
        Serial.println("Error inicializacao do modulo SDcard");
    }
    else
    {
        Serial.println("SDCard encontrado!");
    }

    uint16_t sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;
    allDatas.id = addressIdESP;

    // Loop infinito obrigatorio para manter a Task rodando
    while(true) 
    {

        if (isActiveSDCard) 
        {
            if (sdCardTimeLoopSerial) sdCardTimeLoopSerial--;
            else 
            {
                Serial.println(SD_DEBUG_MESSAGE);
                sdCardTimeLoopSerial = sdCard.sdDebugTimeSerial;
            }
        }

        if ( gravar == 0 )
        {
            if ( isActiveSDCard )
            {
                // Adiciona um novo registro no arquivo "Data_ALL"
                // String filename = "/Data_ALL_Node-" + addressIdESP + ".dat";
                String filename = "/Data_ALL_Node-" + addressIdESP + "_" + allDatas.bootSequence + ".dat";
                allDatas.sequence = conterALL;
                conterALL++;
                allDatas.seconds = esp_timer_get_time()/1000000;
                String payload_all = module.encoderStrAllDatas(allDatas);
                sdCard.appendFile(SD, filename.c_str(), payload_all.c_str());
                wifi.printClientWS("CARD_SAVE: " + payload_all);
            }
            gravar = 1000;
        }
        else
        {
            gravar--;
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(sdCard.sdTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
        // vTaskDelay(pdMS_TO_TICKS(config.sdTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

// Funcao que implementa a Task para leitura do GPS
// Inputs: void
// Return: void
void TaskGPS( void * pvParameters ) 
{
    esp_task_wdt_add(NULL);         // inicia o watchdogtime da task

    //===========================================
    // Inicializa o modulo GPS e retorna TRUE se bem sucedido
    isActiveGPS = gps.begin();

    // Loop infinito obrigatorio para manter a Task rodando
    while( true ) 
    {

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
                if ( !module.isValidUnixTime(module.getUnixTimeNow()) )
                {
                    if ( gps.requestSync() )
                    {
                        validDateTime = true;
                        module.setUnixTime(gps.getEpochTimeNow());

                    }
                    else
                    {
                        validDateTime = false;
                    }                    
                }

                time_t _epochTimeNow = module.getUnixTimeNow();
                allDatas.dateTime = String(month(_epochTimeNow)) + "/" + String(day(_epochTimeNow)) + "/" + String(year(_epochTimeNow)) + " " + String(hour(_epochTimeNow)) + ":" + String(minute(_epochTimeNow)) + ":" + String(second(_epochTimeNow));
                allDatas.epochTimeGPS = _epochTimeNow;      // mudar aqui

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
                    
                    // Verifica se um sdCard esta presente
                    if ( isActiveSDCard )
                    {

                        // Adiciona um novo registro no arquivo "Data_ALL"
                        // String filename = "/Data_ALL_Node-" + addressIdESP + ".dat";
                        String filename = "/Data_ALL_Node-" + addressIdESP + "_" + allDatas.bootSequence + ".dat";

                        allDatas.sequence = conterALL;
                        conterALL++;
                        allDatas.seconds = esp_timer_get_time()/1000000;
                        String payload_all = module.encoderStrAllDatas(allDatas);
                        sdCard.appendFile(SD, filename.c_str(), payload_all.c_str());
                        wifi.printClientWS("CARD_SAVE: " + payload_all);

                        // Adiciona um novo registro no arquivo "Data_GPS"
                        // filename = "/Data_GPS_Node-" + addressIdESP + ".dat";
                        filename = "/Data_GPS_Node-" + addressIdESP + "_" + allDatas.bootSequence + ".dat";

                        String payload_gps = module.encoderJSon(addressIdESP, GPS_TYPE, currentPosition);
                        sdCard.appendFile(SD, filename.c_str(), payload_gps.c_str());
                        wifi.printClientWS("CARD_SAVE: " + payload_gps);

                        gravar = 1000;

                    }

                    // wifi.printClientWS( module.encoderStrAllDatas(allDatas) );
                    Serial.print("GPSN_LOOP: ");
                    Serial.println( module.encoderStrAllDatas(allDatas) );
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
    // disableCore0WDT();              // Desabilita o watchdogtime de hardware do core 0

    //===========================================
    // Inicializacao do modulo IMU (MPU9250)
    Serial.println("Inicializando sensor IMU.....");
    if (!sensorIMU.begin()) 
    {
        isActiveIMU = false;
    } 
    else 
    {
        isActiveIMU = true;
    }

    // EulerAngles_t lastDatasEulerAngles;
    // Loop infinito obrigatorio para manter a Task rodando
    while(true) 
    {

        // IMU
        if ( isActiveIMU ) 
        {
            
            if ( sensorIMU.avaliable() ) 
            {

                if ( sensorIMU.updateDatas() ) 
                {

                    IMU_t _datasIMU = sensorIMU.getIMU();
                    Quaternion_t _datasQuaternion = sensorIMU.getQuaternion();
                    _datasEulerAngles = sensorIMU.getEulerAngles();

                    time_t _epochTimeNow = module.getUnixTimeNow();
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
                    strParametersIMU = sensorIMU.getStrDatas(module.getUnixTimeNow(), _datasIMU, _datasQuaternion, _datasEulerAngles);

                    boolean _newData = false;
                    if ( (_datasEulerAngles.Yam - lastDatasEulerAngles.Yam) > DATA_YAM_DIF ||
                         (_datasEulerAngles.Pitch - lastDatasEulerAngles.Pitch) > DATA_PITCH_DIF ||
                         (_datasEulerAngles.Roll - lastDatasEulerAngles.Roll) > DATA_ROLL_DIF ) 
                    {
                        _newData = true;
                        numberOfRegisterIMU++;

                        // Serial.println("_newData");
                    }
                    lastDatasEulerAngles = _datasEulerAngles;

                    if ( sdCard.isSDCardPresent() && _newData ) 
                    {
                        // String filename = "/Data_ALL_" + String(year()) + String(month()) + String(day()) + "_" + addressIdESP + ".dat";
                        // String filename = "/Data_ALL_Node-" + addressIdESP + ".dat";
                        String filename = "/Data_ALL_Node-" + addressIdESP + "_" + allDatas.bootSequence + ".dat";

                        allDatas.sequence = conterALL;
                        conterALL++;
                        allDatas.seconds = esp_timer_get_time()/1000000;
                        String payload_all = module.encoderStrAllDatas(allDatas);
                        sdCard.appendFile(SD, filename.c_str(), payload_all.c_str());
                        wifi.printClientWS("CARD_SAVE: " + payload_all);

                        // numberOfRegisterIMU++;

                        // filename = "/Data_IMU_Node-" + addressIdESP + ".dat";
                        filename = "/Data_IMU_Node-" + addressIdESP + "_" + allDatas.bootSequence + ".dat";
                        String payload_imu = module.encoderJSon(addressIdESP, IMU_TYPE, strParametersIMU);
                        sdCard.appendFile(SD, filename.c_str(), payload_imu.c_str());
                        wifi.printClientWS("CARD_SAVE: " + payload_imu);

                        gravar = 1000;

                    }
                }
            }
        }

        esp_task_wdt_reset();                               // Reseta o contador do watchdog
        vTaskDelay(pdMS_TO_TICKS(sensorIMU.imuTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
        // vTaskDelay(pdMS_TO_TICKS(config.imuTaskDelayMS));      // Pausa a Tesk e libera o nucleo para a proximo Tesk na fila de prioridade
    }
}

void loop() 
{
  // put your main code here, to run repeatedly:
}