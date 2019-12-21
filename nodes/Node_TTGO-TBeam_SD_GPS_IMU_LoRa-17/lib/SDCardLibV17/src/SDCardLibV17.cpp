#include "SDCardLibV17.h"

// File myFile;
bool SDCardPresent = false;
// String filename;

// SPIClass *spi_sdcard = NULL;
SPIClass spi_sdcard;

SDCardLib::SDCardLib() {}

// Funcao de inicialização do modulo SDCard
// Inputs: nenhum
// Return: bool retorna true se a inicilizacao foi feita com sucesso e false em casa contrario
bool SDCardLib::begin(void) {

    pinMode(SD_PIN_SS, OUTPUT);    

    spi_sdcard.begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_SS);
    spi_sdcard.setFrequency(4000000);

    if (!SD.begin(SD_PIN_SS, spi_sdcard)) {
        return false;
    }

    uint8_t cardType = SD.cardType();
    
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    sdDebugMain = SD_DEBUG_MAIN;
    sdDebugSave = SD_DEBUG_SAVE;
    sdTaskDelayMS = SD_TASK_DELAY_MS;
    sdDebugTimeSerial = SD_DEBUG_TIME_SERIAL/sdTaskDelayMS;

    SDCardPresent = true;
    return true;
}

// Funcao para abrir um arquivo para leitura e/ou escrita no final do arquivo
// Inputs:  String nome do arquivo a ser aberto
//          uint8_t FILE_WRITE (O_READ | O_WRITE | O_CREAT) ou FILE_READ (O_READ) ou FILE_APPEND
// Return: bool retorna true se o arquivo for aberto com sucesso ou false caso contrario
bool SDCardLib::appendFile(fs::FS &fs, const char * filename, const char * message) {

    File myFile = fs.open(filename, FILE_APPEND);

    if (myFile) {
        if (myFile.println(message)) {
            Serial.print("SD_SAVE: ");
            Serial.println(message);
            return true;
        } else {
            Serial.println("SDCard Erro: not write in file");
            return false;
        }
        myFile.close();
    } else {
        Serial.println("SDCard Erro: file not open");
        return false;
    }

    return true;
}

// Funcao para verificar se o SDCard esta presente
// Inputs: nenhum
// Return: bool retorna true se o cartao estiver presente e false em casa contrario
bool SDCardLib::isSDCardPresent(void) {
    return SDCardPresent;
}