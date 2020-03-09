#include "ESP32NodeLibV20.h"

//======================
// Funcoes Public
//======================

// Funcao construtor da classe ESP32Lib
ESP32NodeLib::ESP32NodeLib() {}

// Utiliza o MAC do modulo ESP32 para gerar um ID do modulo
// Input: nenhum
// Return: String - formada com parte do MAC (3 bytes) do modulo
String ESP32NodeLib::getIdModule(void) {
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));       // print High 2 bytes
    Serial.printf("%08X\n",(uint32_t)chipID);                           // print Low 4 bytes.

    uint8_t id8 = (uint8_t)(chipID>>24);
    if (id8 > 9) sprintf(chipID_0, "%0X", id8);
    else sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    if (id16 > 9) sprintf(chipID_1, "%0X", id16);
    else sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    if (id32 > 9) sprintf(chipID_2, "%0X", id32);
    else sprintf(chipID_2, "0%0X", id32);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// Seta a data e hora no RTC interno do ESP32 
// Inputs: unsigned long - valor unix time para a data hora atual
// Return: void
unsigned long ESP32NodeLib::getUnixTimeNow(void)
{
    return now();
}

// Seta a data e hora no RTC interno do ESP32 
// Inputs: unsigned long - valor unix time para a data hora atual
// Return: void
void ESP32NodeLib::setUnixTime(unsigned long unixTime)
{
    setTime( unixTime );
}