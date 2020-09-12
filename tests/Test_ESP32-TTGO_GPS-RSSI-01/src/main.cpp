#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "GPSNeo6MLibV3.h"

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

GPSNeo6MLib gps;

String strParametersGPS;
boolean isActiveGPS = false;

time_t nowTime();

unsigned int counter = 0;

String rssi = "RSSI --";
String packSize = "--";
String packet ;

void setup() {
    pinMode(2,OUTPUT);
    delay(50); 

    
    Serial.begin(115200);
    while (!Serial);
    Serial.println();
    Serial.println("LoRa Sender Test");
  
    SPI.begin(SCK,MISO,MOSI,SS);
    LoRa.setPins(SS,RST,DI0);
    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    // LoRa.onReceive(cbk);
    // LoRa.receive();
    Serial.println("init ok");
   
    delay(1500);

    //===========================================
    // Inicializacao do sensor GPS
    Serial.println("Inicializando GPS.....");
    if (!gps.begin()) {
        isActiveGPS = false;
    } else {
        gps.requestSync();
        isActiveGPS = true;
    }

}

void loop() {
//   display.clear();
//   display.setTextAlignment(TEXT_ALIGN_LEFT);
//   display.setFont(ArialMT_Plain_10);
  
//   display.drawString(0, 0, "Sending packet: ");
//   display.drawString(90, 0, String(counter));
    // Serial.println(String(counter));
    Serial.println(String(counter));
//   display.display();

    if (isActiveGPS) {
        gps.run();
        // send packet
        if (gps.isAvaliableDatas()) {

            String text = gps.getStringDataGPS(nowTime());
            Serial.println(text);

            LoRa.beginPacket();
            // LoRa.print("hello ");
            // LoRa.print(counter);
            LoRa.print(text);
            LoRa.endPacket();
        }
    }

    counter++;
    digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
    delay(100);                       // wait for a second}
}

time_t nowTime(void) {
    if (isActiveGPS) {
        return gps.getUnixTime();
    }
    else {
        return millis();
    }
    
}
