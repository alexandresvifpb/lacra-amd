#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

                                        //D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\ESP32-TTGO-TBeam_Teste-Tramissor-LoRa-01
#define MESSAGE_INITIAL                 ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\ESP32-TTGO-TBeam_Teste-Tramissor-LoRa-01")          // localizacao do projeto

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define VBAT_PIN        35  // GPIO35 - pino para leitura da Vpp da bateria
#define BAND            915E6
#define SYN_WORD        0x00
#define DEBUG_LORAT     true

#define LED_IN_PIN      14      // Blue LED (0=off, 1=on)

time_t nowTime();
boolean isActiveGPS = false;

uint16_t delaySendLoRa = 200;       // intervalo entre um nova transmissao
uint64_t lastTimeLora = millis();
boolean sendPacket = false;

uint8_t ledPin = LED_IN_PIN;
uint16_t delayLed = 1000;           // intervalo entre cada piscada do led Blue
uint64_t lastTimeLed = millis();
boolean isLedHIGH = false;


void setup() {
    pinMode(ledPin,OUTPUT);
    delay(50); 
  
    //===========================================
    // Inicializacao do Porta Serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println();
    Serial.println(MESSAGE_INITIAL);
    Serial.println();

    delay(1500);

    //===========================================
    // Inicializacao do LoRa
    SPI.begin(SCK,MISO,MOSI,SS);
    LoRa.setPins(SS,RST,DI0);
    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    Serial.println("init ok");
   
    delay(1500);
}

void loop() {
    // String text = "[" + String(millis()) + "]";
    String text = "[" + String(nowTime()) + "]";

    if ((lastTimeLora + delaySendLoRa) <= millis()) {
        lastTimeLora = millis();
        sendPacket = true;
    }

    if (sendPacket) {
        LoRa.beginPacket();
        LoRa.print(text);
        LoRa.endPacket();
        Serial.println(text);

        sendPacket = false;
    }

    if ((lastTimeLed + delayLed) <= millis()) {
        lastTimeLed = millis();
        if (isLedHIGH) {
            isLedHIGH = true;
            ledPin = LOW;
        } else {
            isLedHIGH = false;
            ledPin = HIGH;
        }
        Serial.println(text);
    }
}

time_t nowTime(void) {
    if (isActiveGPS) {
        // return gps.getUnixTime();
    }
    else {
        return millis();
    }
    
}