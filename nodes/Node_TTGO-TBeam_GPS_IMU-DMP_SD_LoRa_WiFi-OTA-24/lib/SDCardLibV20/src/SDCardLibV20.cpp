#include "SDCardLibV20.h"

SPIClass spi_sdcard;
boolean SDCardPresent = false;
LinkedList<recordFormatSDCard_t> listSaveRecord = LinkedList<recordFormatSDCard_t>();

SDCardLib::SDCardLib() {}

// SDCard module initialization
boolean SDCardLib::begin(void) {

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

    SDCardPresent = true;
    return true;
}

//
void SDCardLib::run(void) {
    while ( listSaveRecord.size() > 0 ) {
        String filename = "";
        recordFormatSDCard_t record = listSaveRecord.remove(0);
        String strID = record.id.substring(4);
        switch (record.type)
        {
        case TYPE_ALL:
            // filename = String("/Data_ALL_Node-" + record.id + '_' + record.bootSequence + ".dat").c_str();
            filename = "/ALL" + strID + String(record.bootSequence) + ".dat";
            break;
        
        case TYPE_IMU:
            // filename = String("/Data_IMU_Node-" + record.id + '_' + record.bootSequence + ".dat").c_str();
            // filename = String("/IMU88" + record.bootSequence + ".dat").c_str();
            filename = "/IMU" + strID + String(record.bootSequence) + ".dat";
            break;
        
        case TYPE_GPS:
            // filename = String("/Data_GPS_Node-" + record.id + '_' + record.bootSequence + ".dat").c_str();
            // filename = String("/GPS88" + record.bootSequence + ".dat").c_str();
            filename = "/GPS" + strID + String(record.bootSequence) + ".dat";
            break;
        
        default:
            break;
        }
        appendFile(SD, filename.c_str(), record.payload.c_str());

        Serial.print("CARD_SAVE: ");
        Serial.println(record.type);
    }
}

// check if the SDCard is present
boolean SDCardLib::isSDCardPresent(void) {
    
    // File configFile = SD.open("/config.dat", FILE_READ);
    // if ( configFile.available() ) {
    //     SDCardPresent = true;
    // } else {
    //     SDCardPresent = begin();
    // }
    // configFile.close();

    // ( SDCardPresent ) ? Serial.println("SDCardPresent: true") : Serial.println("SDCardPresent: false");

    return SDCardPresent;
}

//
boolean SDCardLib::addRecord(recordFormatSDCard_t value) {
    return listSaveRecord.add(value);
}

// open a file for reading and / or writing at the end of the file
boolean SDCardLib::appendFile(fs::FS &fs, const char * filename, const char * message) {

    File myFile = fs.open(filename, FILE_APPEND);

    if (myFile) {
        if (myFile.println(message)) {
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
