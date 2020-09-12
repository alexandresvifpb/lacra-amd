#ifndef SDCARDLIBV20_H 
#define SDCARDLIBV20_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "LinkedList.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_PIN_SCK              (14)
#define SD_PIN_MISO             (2)
#define SD_PIN_MOSI             (13)
#define SD_PIN_SS               (25)

#define SD_TASK_DELAY_MS        (1)

typedef struct {
    String id;
    uint16_t bootSequence;
    uint8_t type;
    String payload;
} recordFormatSDCard_t;

enum Tscale {
  TYPE_ALL = 0,
  TYPE_IMU,
  TYPE_GPS,
};

class SDCardLib
{
    public:
        SDCardLib();                // construtor

        boolean begin(void);
        void run(void);
        boolean isSDCardPresent(void);
        boolean addRecord(recordFormatSDCard_t value);
        boolean appendFile(fs::FS &fs, const char * filename, const char * message);
        // String readSdConfig(void);

        // boolean sdDebugMain;
        // boolean sdDebugSave;
        // uint16_t sdDebugTimeSerial;
        uint32_t tsSDCardTaskDelayMS = SD_TASK_DELAY_MS;

    private:


};

#ifdef __cplusplus
}
#endif

#endif  // SDCARDLIBV20_H