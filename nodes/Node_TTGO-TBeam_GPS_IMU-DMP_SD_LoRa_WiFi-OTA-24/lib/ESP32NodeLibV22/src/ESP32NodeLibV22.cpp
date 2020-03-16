#include "ESP32NodeLibV22.h"

uint16_t getFlashUInt16(uint8_t address);
void setFlashUInt16(uint8_t address, uint16_t value);
unsigned long timeLocal;

// function constructor of the class ESP32Lib
ESP32NodeLib::ESP32NodeLib() {}

// uses the MAC of the ESP32 for generate the device ID
String ESP32NodeLib::getIdModule(void) {
    char chipID_0[4];
    char chipID_1[4];
    char chipID_2[4];
    uint64_t chipID = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipID>>32));
    Serial.printf("%08X\n",(uint32_t)chipID);

    uint8_t id8 = (uint8_t)(chipID>>24);
    (id8 > 9) ? sprintf(chipID_0, "%0X", id8) : sprintf(chipID_0, "0%0X", id8);

    uint16_t id16 = (uint8_t)(chipID>>32);
    (id16 > 9) ? sprintf(chipID_1, "%0X", id16) : sprintf(chipID_1, "0%0X", id16);

    uint32_t id32 = (uint8_t)(chipID>>40);
    (id32 > 9) ? sprintf(chipID_2, "%0X", id32) : sprintf(chipID_2, "0%0X", id32);

    return String(chipID_0) + String(chipID_1) + String(chipID_2);
}

// get next value of the sequence boot and update in the EEPROM
uint16_t ESP32NodeLib::getBootSequence(void) {
    // reset the bootsequence variable
    if ( RESET_CONT_BOOT ) {
        setFlashUInt16(0, 0);
        EEPROM.commit();
        Serial.print("EEPROM.read(): ");
    } else {
        uint16_t _bootseq = getFlashUInt16(0);
        setFlashUInt16(0, _bootseq + 1);
        EEPROM.commit();
        return _bootseq;
    }
}

// Get the date and time on the ESP32 internal RTC
unsigned long ESP32NodeLib::getUnixTimeNow(void) {
    // unsigned long tempLocal = timeLocal + millis();
    return timeLocal + millis()/1000;
    // timeval tv_now;
    // gettimeofday(&tv_now, NULL);
    // return tv_now.tv_sec;
}

// Set the date and time on the ESP32 internal RTC 
void ESP32NodeLib::setUnixTime(unsigned long unixTime) {

    timeLocal = unixTime - millis()/1000;

   	// timeval tv;                     // Creates the temporary structure for the function below.
    // tv.tv_sec = unixTime - 1073;    // Assign my current date. You can use NTP for this or any other source.
	// settimeofday(&tv, NULL);        // Configures the RTC to keep the assigned date up to date.
}

// Checks whether the current date is within the valid range
boolean ESP32NodeLib::isValidUnixTime(uint64_t currentUnixTime) {
    boolean rest = false;
    ( (currentUnixTime > minUnixTime) && (currentUnixTime < maxUnixTime) ) ? rest = true : rest = false;
    return rest;
}

// Read from flash memory
uint16_t getFlashUInt16(uint8_t address) {
    return word(EEPROM.read(address), EEPROM.read(address + 1));
}

// Writes to flash memory
void setFlashUInt16(uint8_t address, uint16_t value) {
    EEPROM.write(address,highByte(value));
    EEPROM.write(address + 1,lowByte(value));
}
