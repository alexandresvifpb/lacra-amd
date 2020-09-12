#include <Arduino.h>
#include "HRSpO2LibV24.h"

HRSPO2Lib sensor_hrspo2;
long lastTime = millis();

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  sensor_hrspo2.begin();
}

void loop() {
  
  sensor_hrspo2.run();

  
/*
  if ( sensor_hrspo2.avaliable() ) {
    HRSPO2_t currentSample = sensor_hrspo2.getHRSpO2();

    String strDatas = "BPM: ";
    strDatas += String(currentSample.beatsPerMinute);
    strDatas += ", avgBPM: ";
    strDatas += String(currentSample.beatAvg);
    strDatas += ", Temp: ";
    strDatas += String(currentSample.temperature);
    strDatas += ", Valid Sample: ";
    currentSample.validSample ? strDatas += "True": strDatas += "False"; 

    Serial.println(strDatas);
  }
  */

}