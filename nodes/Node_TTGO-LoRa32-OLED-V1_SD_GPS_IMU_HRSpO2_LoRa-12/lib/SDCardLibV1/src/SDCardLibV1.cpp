#include "SDCardLibV1.h"

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

    pinMode(SD_SS_PIN, OUTPUT);    

    spi_sdcard.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_SS_PIN);
    spi_sdcard.setFrequency(4000000);

    // if (!fs.begin(SD_SS_PIN, spi_sdcard)) {
    if (!SD.begin(SD_SS_PIN, spi_sdcard)) {
        return false;
    }

    uint8_t cardType = SD.cardType();
    
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
    //     return;
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

    // appendFile(SD, "/Teste_b.dat", "strParametersIMU");

    SDCardPresent = true;
    return true;
}

/*
// Funcao para abrir um arquivo para leitura e/ou escrita no final do arquivo
// Inputs:  String nome do arquivo a ser aberto
//          uint8_t FILE_WRITE (O_READ | O_WRITE | O_CREAT) ou FILE_READ (O_READ) ou FILE_APPEND
// Return: bool retorna true se o arquivo for aberto com sucesso ou false caso contrario
bool SDCardLib::openFile(String filename, uint8_t mode) {

    if (myFile) myFile.close();

    myFile = SD.open(filename.c_str, mode);
    
    if (!myFile) return false;

    return true;
}
*/

// Funcao para abrir um arquivo para leitura e/ou escrita no final do arquivo
// Inputs:  String nome do arquivo a ser aberto
//          uint8_t FILE_WRITE (O_READ | O_WRITE | O_CREAT) ou FILE_READ (O_READ) ou FILE_APPEND
// Return: bool retorna true se o arquivo for aberto com sucesso ou false caso contrario
bool SDCardLib::appendFile(fs::FS &fs, const char * filename, const char * message) {

    File myFile = fs.open(filename, FILE_APPEND);

    if (myFile) {
        if (myFile.println(message)) {
            Serial.print("SDCard Save: ");
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

// Funcao para criar uma string codificada em JSON
// Inputs:  String id - endereco do dispositivo
//          uint8_t type - tipo do dados a ser salvo
//          String payload - dados a ser salvo
// Return: String retorna uma string com os dados codificado em JSON
String SDCardLib::encoderJSon(String id, uint8_t type, String payload) {

    // messageSDCard_t _lineSDCard;
    // _lineSDCard.id = id;
    // _lineSDCard.type = type;
    // _lineSDCard.payload = payload;
    
    String _strJSON;

    StaticJsonDocument<256> doc;

    JsonObject root = doc.to<JsonObject>();

    // root["id"] = msgSend.id;
    // root["type"] = msgSend.type;
    // root["payload"] = msgSend.payload;

    root["id"] = id;
    root["type"] = type;
    root["payload"] = payload;

    if (serializeJson(doc, _strJSON) == 0) {
        Serial.println(F("Failed to write to file"));
    }

    return _strJSON;

}

// Funcao para verificar se o SDCard esta presente
// Inputs: nenhum
// Return: bool retorna true se o cartao estiver presente e false em casa contrario
bool SDCardLib::isSDCardPresent(void) {
    return SDCardPresent;
}