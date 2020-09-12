#ifndef ESP32GATEWAYLIBV1_H 
#define ESP32GATEWAYLIBV1_H

#include "ArduinoJson.h"
#include "LinkedList.h"
#include "TimeLib.h"
#include "EEPROM.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESP32_TIME_DELAY            (120000)            // em mili segundos

const uint64_t minUnixTime = 1577840400000;             // 01/01/2020 00:00:00
const uint64_t maxUnixTime = 1609462800000;             // 01/01/2021 00:00:00

// struct timeval data;          //Cria a estrutura que contem as informacoes da data.

#define RESET_CONT_BOOT             (false)

typedef struct {
    String id;
    String dateTime;
    uint64_t epochTimeIMU;
    uint16_t bootSequence;
    uint64_t sequence;
    uint64_t seconds;
} configs_t;

typedef struct  
{
    uint8_t type;
    String payload;
} commands_t;

class ESP32GatewayLib
{
    public:
        ESP32GatewayLib();                         // construtor

        String getIdModule(void);

        unsigned long getUnixTimeNow(void);
        void setUnixTime(unsigned long unixTime);
        boolean isValidUnixTime(uint64_t currentUnixTime);

        String encoderJSon(String id, uint8_t type, String payload);
        String encoderStrConfigs(configs_t datas);
        commands_t getCommand(String sentence);
        uint16_t getBootSequence(void);

        uint64_t timeTaskDelayMS = ESP32_TIME_DELAY;

    private:
        timeval tv;                             //Cria a estrutura temporaria para funcao abaixo.


};


#ifdef __cplusplus
}
#endif

#endif  // ESP32GATEWAYLIBV1_H