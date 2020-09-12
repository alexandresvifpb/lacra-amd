#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

                                        //D:\downloads\doutorado\2019\prototipos\firmwares\lacra-amd\ESP32-TTGO-OLED_Teste-Receptor-LoRa-01
#define MESSAGE_INITIAL                 ("D:\\downloads\\doutorado\\2019\\prototipos\\firmwares\\lacra-amd\\ESP32-TTGO-OLED_Teste-Receptor-LoRa-01")          // localizacao do projeto

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

//===========================================
// LoRa
int rssi;
float snr;
String packSize = "--";
String packet ;

void loraData();
void cbk(int packetSize);
String parseData(String textRcv, int _rssi, float _snr);

void setup() {
    pinMode(16,OUTPUT);
    digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
    delay(50); 
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  
  
    //===========================================
    // Inicializacao do Porta Serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println();
    Serial.println(MESSAGE_INITIAL);

    delay(500);

    //===========================================
    // Inicializacao do LoRa
    SPI.begin(SCK,MISO,MOSI,SS);
    LoRa.setPins(SS,RST,DI0);  
    if (!LoRa.begin(BAND)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }
    LoRa.onReceive(cbk);
    LoRa.receive();
    Serial.println("init ok");
   
    delay(1500);
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) { cbk(packetSize);  }
    delay(10);
}

void loraData() {
    // String text = "Received " + packSize + " bytes, " + "message: " + packet + rssi;
    String text = packet + ";" + String(rssi, DEC) + ";" + String(snr);
    // text =+ packSize;
    // Serial.println(text);
    // Serial.println(rssi);
}

void cbk(int packetSize) {
    packet ="";
    packSize = String(packetSize,DEC);
    for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
    // rssi = " RSSI " + String(LoRa.packetRssi(), DEC) ;
    rssi = LoRa.packetRssi();
    snr = LoRa.packetSnr();

    // loraData();
    Serial.println(parseData(packet, rssi, snr));
}

String parseData(String textRcv, int _rssi, float _snr) {
    textRcv.replace("]",";3");
    textRcv.concat(String(_rssi, DEC));
    textRcv.concat(";");
    textRcv.concat(String(_snr));
    textRcv.concat("]");
    return textRcv;
}