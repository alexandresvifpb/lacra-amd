#include <Arduino.h>
#include <ArduinoJson.h>
#include <LinkedList.h>
#include "LoRa.h"

#ifndef LORANODELIBV3_h 
#define LORANODELIBV3_h

#ifdef __cplusplus
extern "C" {
#endif

#define SCK_PIN         5    // GPIO5  -- SX1278's SCK
#define MISO_PIN        19   // GPIO19 -- SX1278's MISO
#define MOSI_PIN        27   // GPIO27 -- SX1278's MOSI
#define SS_PIN          18   // GPIO18 -- SX1278's CS
#define RST_PIN         14   // GPIO14 -- SX1278's RESET
#define DIO0_PIN        26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define VBAT_PIN        35  // GPIO35 - pino para leitura da Vpp da bateria

#define BAND            915E6       //433E6  //you can set band here directly,e.g. 868E6,915E6
#define SYN_WORD        0x00
#define DEBUG_LORAT     true
// #define SEND_INTERVAL   500     // milesegundos

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageLoRa_t;

class LoRaNodeLib
{
    public:
        LoRaNodeLib();            // construtor

        bool begin(void);
        void run(void);
        bool send(String strSend);
        bool isReceived(void);
        String getStrRecv(void);
        messageLoRa_t getMsgRecv(void);
        String getLocalAddress(void);
        // void addNewMsg(String strNewMsg);
        void addNewMsg(String strNewMsg, uint8_t type);
        uint8_t isMsgForSend(void);
        float getVBat(void);

    private:
        static void onReceive(int packetSize);
        void SetRxMode(void);
        void SetTxMode(void);
        String encodeJSON(messageLoRa_t msgSend);
        // String encodeJSON(messageLoRa_t msgSend, uint8_t type);
        static messageLoRa_t decodeJSON(String strJSONRecv);

        uint8_t synWord = SYN_WORD;
        const uint8_t vbatPin = VBAT_PIN;
        // const uint8_t sendInterval = SEND_INTERVAL;
};

#ifdef __cplusplus
}
#endif

#endif  // LORANODELIBV3_h