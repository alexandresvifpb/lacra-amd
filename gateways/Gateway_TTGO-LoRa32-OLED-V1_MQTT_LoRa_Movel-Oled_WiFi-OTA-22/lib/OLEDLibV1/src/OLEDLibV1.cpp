#include "OLEDLibV1.h"

SSD1306 display(0x3c, OLED_PIN_SDA, OLED_PIN_SCL);
messageSend_t listSendText;

//======================
// Funcoes Public
//======================

OLEDLib::OLEDLib() {}

boolean OLEDLib::begin(void) 
{
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Gateway LoRa V22");
    display.display();
    listSendText.loraMessageText = "LoRa";
    listSendText.otaMessageText = "OTA";
    tsOledTaskDelayMS = OLED_TASK_DELAY_MS;
    return true;
}

void OLEDLib::run(void) 
{
    display.clear();
    display.drawString(0, 0, listSendText.loraMessageText);
    display.drawString(0, 10, listSendText.otaMessageText);
    display.display();
}

void OLEDLib::setLoRaMessageShowOledScreen(String text)
{
    listSendText.loraMessageText = text;
}

void OLEDLib::setOTAMessageShowOledScreen(String text)
{
    listSendText.otaMessageText = text;
}

void OLEDLib::print(uint16_t column, uint16_t row, String text)
{
    display.drawString(column, row, text);
    display.display();
}

void OLEDLib::clear(void)
{
    display.clear();
}
