#ifndef OLEDLIBV1_H 
#define OLEDLIBV1_H

#include "Arduino.h"
#include "Wire.h"
#include "SSD1306.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_PIN_SDA            (4)
#define OLED_PIN_SCL            (15)
#define OLED_TASK_DELAY_MS      (500)

typedef struct {
    String loraMessageText;
    String otaMessageText;
} messageSend_t;

class OLEDLib
{
    public:
        OLEDLib();                         // construtor

        boolean begin(void);
        void run(void);
        void setLoRaMessageShowOledScreen(String text);
        void setOTAMessageShowOledScreen(String text);
        void print(uint16_t column, uint16_t row, String text);
        void clear(void);

        uint32_t tsOledTaskDelayMS;

    private:


};


#ifdef __cplusplus
}
#endif

#endif  // OLEDLIBV1_H