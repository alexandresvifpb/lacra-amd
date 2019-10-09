#include <Arduino.h>

// Serial                                 D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\gateways\Gateway_TTGO_LoRa32-OLED_V1_OTA_WiFi_MQTT_LoRa-15
#define MESSAGE_INITIAL                 ("C:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\getways\\Gateway_TTGO_LoRa32-OLED_V1_OTA_WiFi_MQTT_LoRa-15")          // localizacao do projeto
#define DEBUG                           (true)         // variavel que habilita (1) ou desabilita (0) o envio de dados pela seria para realizar debug no programa

TaskHandle_t Task0;
TaskHandle_t Task1;

// time_t nowTime();
void Task0code( void * pvParameters );
void Task1code( void * pvParameters );

//===========================================
//  Watchdog Timer
#include "esp_system.h"

#define   WATCHDOG_TIME_OUT_SEGUND  (30)    // tempo em segundos para ativar o watchdog

const int wdtTimeout = WATCHDOG_TIME_OUT_SEGUND * 1000;           //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;
TickType_t taskDelay = 5000 / portTICK_PERIOD_MS;

void resetWatchdog(void);

void IRAM_ATTR resetModule() {
  ets_printf("Reboot Watch Dog Time\n");
  esp_restart();
}

//===========================================
// MQTT
#include <MQTTLibV2.h>

MQTTLib mqttClient;
bool isWiFiConnected = false;
bool isMQTTConnected = false;

bool sendMQTT = false;
String strSendMQTT;
uint8_t numberOfReconnectAttempts = 10;

//===========================================
// LoRa
#include "LoRaGatewayLibV4.h"

LoRaGatewayLib lora = LoRaGatewayLib();
bool isWorking_LoRa = false;
String recvLoRaString;

#include <LinkedList.h>
LinkedList<String> listMessageSendMQTT = LinkedList<String>();
uint16_t numberMessageSendMQTT = 0;

void setup() {
    // Inicializacao da Serial
    Serial.begin(115200);
    Serial.println();
    Serial.println(MESSAGE_INITIAL);
    Serial.println();

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                    Task0code,   /* Task function. */
                    "Task0",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task0,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
    delay(500); 

    //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 

}
//Task0code: 
void Task0code( void * pvParameters ) {
    Serial.print("Task0 running on core ");
    Serial.println(xPortGetCoreID());

    //===========================================
    //  Watchdog Timer
    timer = timerBegin(0, 80, true);                  //timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true);  //attach callback
    timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
    timerAlarmEnable(timer);                          //enable interrupt

    //===========================================
    // WiFi
    isWiFiConnected = mqttClient.begin();
    isMQTTConnected = mqttClient.init();

    for(;;) {

        resetWatchdog();        // reinicializa o watchdog

        // uint32_t timenow = millis();

        // if (mqttClient.isConnected()) {
        if (mqttClient.checkWiFiConnection()) {
            uint8_t _cont = 10;

            // while (numberMessageSendMQTT && listMessageSendMQTT.size() && isMQTTConnected && _cont) {
            while (numberMessageSendMQTT && listMessageSendMQTT.size() && _cont) {

                mqttClient.send(listMessageSendMQTT.remove(0));
                _cont++;
                delay(10);

                // Serial.print("loop: ");
                // Serial.println(listMessageSendMQTT.size());
    
                resetWatchdog();        // reinicializa o watchdog
            }
        } else {
            if (!mqttClient.connectWiFi() && (numberOfReconnectAttempts > 0)) {
                numberOfReconnectAttempts--;
            } else {
                esp_restart();
            }

            mqttClient.reconnect();
        }

        mqttClient.loop();

        // vTaskDelay(taskDelay);
        // vTaskDelay(1);
        delay(1);
    }
}

//Task1code: 
void Task1code( void * pvParameters ) {
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());

    //===========================================
    // LoRa
    isWorking_LoRa = lora.begin();;
    if (!isWorking_LoRa)
        Serial.println("Error inicializacao do modulo LoRa");
    else
        Serial.println("Modulo LoRa OK!");
    delay(100);

    for(;;) {

        resetWatchdog();

        if (lora.isReceived()) {
            // recvLoRaString = lora.getStrRecv();
            // wifi.mqtt_send(recvLoRaString);
            String strRecv = lora.getStrRecv();
            listMessageSendMQTT.add(strRecv);
            numberMessageSendMQTT++;

            // Serial.println(listMessageSendMQTT.size());
            // Serial.println(strRecv);
        }

        // vTaskDelay(taskDelay);
        // vTaskDelay(1);
        delay(1);
    } 
}

//===========================================
//  Watchdog Timer
void resetWatchdog(void) {
    timerWrite(timer, 0); //reset timer (feed watchdog)
    return;  
}

void loop() {
  // put your main code here, to run repeatedly:
}