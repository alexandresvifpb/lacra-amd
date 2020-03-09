#ifndef ESP32NODELIBV20_H 
#define ESP32NODELIBV20_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

class ESP32NodeLib
{
    public:
        ESP32NodeLib();                         // construtor

        String getIdModule(void);

        unsigned long getUnixTimeNow(void);
        void setUnixTime(unsigned long unixTime);

    private:
        timeval tv;                             //Cria a estrutura temporaria para funcao abaixo.


};


#ifdef __cplusplus
}
#endif

#endif  // ESP32NODELIBV20_H