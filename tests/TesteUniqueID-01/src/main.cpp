//
// ArduinoUniqueID8.ino
//
// Example shows the last eight UniqueID on the Serial Monitor.
//
#include <Arduino.h>
#include "ArduinoUniqueID.h"

void setup()
{
	Serial.begin(115200);
	UniqueIDdump(Serial);
	Serial.print("UniqueID: ");
	for (size_t i = 0; i < UniqueIDsize; i++)
	{
		if (UniqueID[i] < 0x10)
			Serial.print("0");
		Serial.print(UniqueID[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
}

void loop()
{
}
