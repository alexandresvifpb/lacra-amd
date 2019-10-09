#include <Arduino.h>
#include <ArduinoJson.h>
// #include <SPI.h>
// #include <SD.h>
// #include "mySD.h"

// #include "FS/src/FS.h"
// #include "SD/src/SD.h"
// #include "SPI/src/SPI.h"

#include "FS.h"
#include "SD.h"
#include "SPI.h"

#ifndef SDCARDLIBV1_h 
#define SDCARDLIBV1_h

#ifdef __cplusplus
extern "C" {
#endif

#define SD_SCK_PIN         14
#define SD_MISO_PIN        2
#define SD_MOSI_PIN        13
#define SD_SS_PIN          25

// #define FILE_READ   (O_READ)
// #define FILE_WRITE  (O_READ | O_WRITE | O_CREAT)
// #define FILE_APPEND (O_APPEND)

#define VBAT_PIN        35  // GPIO35 - pino para leitura da Vpp da bateria

typedef struct {
    String id;
    uint8_t type;
    String payload;
} messageSDCard_t;

class SDCardLib
{
    public:
        SDCardLib();            // construtor

        bool begin(void);
        // bool openFile(String filename, uint8_t mode);
        bool appendFile(fs::FS &fs, const char * filename, const char * message);
        String encoderJSon(String id, uint8_t type, String payload);
        bool isSDCardPresent(void);

    private:

        // bool bFileOpen = false;

};

#ifdef __cplusplus
}
#endif

#endif  // SDCARDLIBV1_h