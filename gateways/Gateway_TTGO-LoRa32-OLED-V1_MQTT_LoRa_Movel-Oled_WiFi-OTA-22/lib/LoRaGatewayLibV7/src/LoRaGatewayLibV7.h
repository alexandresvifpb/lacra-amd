#include <Arduino.h>
#include <ArduinoJson.h>
#include "LoRa.h"

#ifndef LORAGATEWAYLIBV7_H 
#define LORAGATEWAYLIBV7_H

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_PIN_SCK            (5)                 // GPIO5  -- SX1278's SCK
#define LORA_PIN_MISO           (19)                // GPIO19 -- SX1278's MISO
#define LORA_PIN_MOSI           (27)                // GPIO27 -- SX1278's MOSI
#define LORA_PIN_SS             (18)                // GPIO18 -- SX1278's CS
#define LORA_PIN_RST            (14)                // GPIO14 -- SX1278's RESET
#define LORA_PIN_DIO0           (26)                // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define LORA_PIN_VBAT           (35)                // GPIO35 - pino para leitura da Vpp da bateria
#define LORA_BAND               (915E6)             //433E6  //you can set band here directly,e.g. 868E6,915E6
#define LORA_SYN_WORD           (0x00)

#define LORA_DEBUG_MAIN         (false)
#define LORA_DEBUG_GETS         (false)
#define LORA_DEBUG_SEND_RECV    (true)
#define LORA_DEBUG_TIME_SERIAL  (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define LORA_DEBUG_MESSAGE      ("LORA_TASK: ")
#define LORA_TASK_DELAY_MS      (1)

typedef struct {
    String id;
    uint8_t type;
    String payload;
    int16_t rssi;
    float snr;
} messageLoRa_t;

class LoRaGatewayLib
{
    public:
        LoRaGatewayLib();            // construtor

        boolean begin(void);
        boolean send(String strSend);
        boolean isReceived(void);
        String getStrRecv(void);
        messageLoRa_t getMsgRecv(void);
        String getLocalAddress(void);

        boolean loraDebugMain;
        boolean loraDebugGetStrRecv;
        uint16_t loraDebugTimeSerial;
        uint32_t loraTaskDelayMS;

    private:
        static void onReceive(int packetSize);
        void SetRxMode(void);
        void SetTxMode(void);
        String encodeJSON(messageLoRa_t msgSend, uint8_t type);
        static messageLoRa_t decodeJSON(String strJSONRecv);

        uint8_t synWord = LORA_SYN_WORD;
};

#ifdef __cplusplus
}
#endif

#endif  // LORAGATEWAYLIBV7_H