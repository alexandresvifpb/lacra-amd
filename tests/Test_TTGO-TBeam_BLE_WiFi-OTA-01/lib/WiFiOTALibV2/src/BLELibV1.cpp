#include "BLELibV1.h"

SimpleBLE ble;

// Construtor da classe WiFiOTALib
BLELib::BLELib() {}

//======================
// Funcoes Public
//======================

// Inicializa o modulo BLE Bluetooth 
// Inputs: void
// Return: boolean - TRUE caso seja estabelicida uma conexao WiFi
boolean BLELib::begin(void)
{
    return ble.begin("ESP32 SimpleBLE");
}

void BLELib::print(String strValue)
{
    Serial.println(strValue);
    ble.begin(strValue);
}