#ifndef LORANODELIBV21_H 
#define LORANODELIBV21_H

#include <ArduinoJson.h>
#include "LoRa.h"
#include "LinkedList.h"
#include "TimeLib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_PIN_SCK                (5)                 // GPIO5  -- SX1278's SCK
#define LORA_PIN_MISO               (19)                // GPIO19 -- SX1278's MISO
#define LORA_PIN_MOSI               (27)                // GPIO27 -- SX1278's MOSI
#define LORA_PIN_SS                 (18)                // GPIO18 -- SX1278's CS
#define LORA_PIN_RST                (14)                // GPIO14 -- SX1278's RESET
#define LORA_PIN_DIO0               (26)                // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_BAND                   (915E6)             //433E6  //you can set band here directly,e.g. 868E6,915E6
#define LORA_SYN_WORD               (0x00)
#define LORA_TASK_DELAY_MS          (1)
#define LORA_DELAY_SEND_RECORD      (5000)

typedef struct {
    String id;
    uint8_t type;
    String payload;
} recordFormatLoRa_t;

class LoRaNodeLib
{
    public:
        LoRaNodeLib();            // construtor

        boolean begin(void);
        void run(void);
        boolean waitForNextSend(void);
        boolean addRecordToSend(recordFormatLoRa_t value);
        boolean send(String strSend);
        boolean isReceived(void);
        String getStrRecv(void);

        uint32_t tsLoRaTaskDelayMS = LORA_TASK_DELAY_MS;

    private:
        String encodeJSON(recordFormatLoRa_t msgSend);
        static void onReceive(int packetSize);
        void SetRxMode(void);
        void SetTxMode(void);

        uint64_t lastSendTime = millis();
        uint32_t tsLoRaDelaySendRecord = LORA_DELAY_SEND_RECORD;

};

#ifdef __cplusplus
}
#endif

#endif  // LORANODELIBV21_H