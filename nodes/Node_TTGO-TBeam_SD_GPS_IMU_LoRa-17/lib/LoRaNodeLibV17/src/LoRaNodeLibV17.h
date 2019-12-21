#include "ESP32LibV17.h"
#include <ArduinoJson.h>
#include "LoRa.h"

#ifndef LORANODELIBV17_H 
#define LORANODELIBV17_H

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
#define LORA_DEBUG_SEND_RECV    (true)
#define LORA_DEBUG_TIME_SERIAL  (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define LORA_DEBUG_MESSAGE      ("LORA_TASK: ")
#define LORA_TASK_DELAY_MS      (1)

#define LORA_UTC_OFFSET         (-3)
#define LORA_EPOCH_TIME_2019    (1546300800)        // 00:00:00 01/01/2019

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageLoRa_t;

class LoRaNodeLib
{
    public:
        LoRaNodeLib();            // construtor

        boolean begin(void);
        boolean send(String strSend);
        boolean isReceived(void);
        String getStrRecv(void);
        messageLoRa_t getMsgRecv(void);
        float getVBat(void);
        boolean requestSyncTime(void);
        void setEpochTime(time_t epochTime);

        boolean loraDebugMain;
        boolean loraDebugSend;
        uint16_t loraDebugTimeSerial;
        uint16_t loraTaskDelayMS;

    private:
        static void onReceive(int packetSize);
        void SetRxMode(void);
        void SetTxMode(void);
        String encodeJSON(messageLoRa_t msgSend);
        static messageLoRa_t decodeJSON(String strJSONRecv);

        uint8_t synWord = LORA_SYN_WORD;
        const uint8_t vbatPin = LORA_PIN_VBAT;
        // String macAddress;
        const int UTC_offset = LORA_UTC_OFFSET;
};

#ifdef __cplusplus
}
#endif

#endif  // LORANODELIBV17_H