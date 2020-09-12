#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

#define ALPHA 0.95  //dc filter alpha value

typedef struct {
  float y;
  float w;
} dcFilter_t;

MAX30105 particleSensor;

dcFilter_t dcRemovalFloat(float x, float w, float alpha);
boolean checkForBeat(float signal);

dcFilter_t dcFilterIR;
boolean beat = false;
float irACSignalPrevious = 0;
float irACSignalCurrent = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 20; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.softReset();
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

}

void loop()
{
  int32_t irValue = particleSensor.getIR();
  if (irValue > 5000) {
    dcFilterIR = dcRemovalFloat((float)irValue, dcFilterIR.w, ALPHA);

    if (checkForBeat(dcFilterIR.y) == true) {
      beat = !beat;
    }

    String strDatas = String(dcFilterIR.y);

    // strDatas += ',';
    // if (beat)
    //   strDatas += "100.00";
    // else
    //   strDatas += "0.00";    
      
    // strDatas += String(meanDiffResIR);
    // strDatas += ',';
    // strDatas += String(beatsPerMinute);

    Serial.println(strDatas);

  }
}

//=======================================
// Functions
//=======================================

// Filter IIR
// x: current input sample,
// w: intermediate value that acts like a history of the DC value of the signal, 
// alpha: scale factor that widens or narrows the filter
dcFilter_t dcRemovalFloat(float x, float w, float alpha) {
  dcFilter_t filtered;
  filtered.w = x + alpha*w;
  filtered.y = filtered.w - w;
  return filtered;
}

//
boolean checkForBeat(float signal) {
  bool beatDetected = false;



  return beatDetected;
}