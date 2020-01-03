#include "ESP32NodeLibV19.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#ifndef SDCARDLIBV19_H 
#define SDCARDLIBV19_H

#ifdef __cplusplus
extern "C" {
#endif

#define SD_PIN_SCK              (14)
#define SD_PIN_MISO             (2)
#define SD_PIN_MOSI             (13)
#define SD_PIN_SS               (25)
#define SD_PIN_VBAT             (35)                // GPIO35 - pino para leitura da Vpp da bateria

#define SD_DEBUG_MAIN           (true)
#define SD_DEBUG_SAVE           (false)
#define SD_DEBUG_TIME_SERIAL    (2000)              // intervalo entre cada envio de uma string pela porta serial para debug da lib
#define SD_DEBUG_MESSAGE        ("SDCARD_TASK: ")
#define SD_TASK_DELAY_MS        (1)

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageSDCard_t;

class SDCardLib
{
    public:
        SDCardLib();                // construtor

        bool begin(void);
        bool appendFile(fs::FS &fs, const char * filename, const char * message);
        // String encoderJSon(String id, uint8_t type, String payload);
        bool isSDCardPresent(void);
        String readSdConfig(void);

        boolean sdDebugMain;
        boolean sdDebugSave;
        uint16_t sdDebugTimeSerial;
        uint16_t sdTaskDelayMS;

    private:


};

#ifdef __cplusplus
}
#endif

#endif  // SDCARDLIBV19_H