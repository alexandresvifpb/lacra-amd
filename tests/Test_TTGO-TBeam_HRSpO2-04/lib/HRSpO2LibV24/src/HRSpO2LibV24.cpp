#include "HRSpO2LibV24.h"

MAX30105 particleSensor;

dcFilter_t dcFilterIR;
boolean beat = false;
float IR_AC_Signal_Previous = 0;
float IR_AC_Signal_Current = 0;
float IR_AC_Signal_Min = 0;
float IR_AC_Signal_Max = 0;
float IR_AC_Max = IR_AC_INIT;
float IR_AC_Min = -IR_AC_INIT;

int16_t positive_Edge = 0;
int16_t negative_Edge = 0;
int16_t IR_avg_reg = 0;

// Butterworth Filter
float valueBwF[2];

// run()
HRSPO2_t current_HRSpO2;
long lastBeatTime = 0;              //Time at which the last beat occurred
long samplesDelay = HRSPO2_SAMPLE_DELAY_MS;
uint8_t counterDelay = SAMPLES_FAULT_COUNTER;
boolean newValidSample = false;

// Filter mediaBPMFilter() 
uint8_t rates[RATE_SIZE];           //Array of heart rates
uint8_t rateSpot = 0;

// Auto tune brightness IR
uint8_t brightnessIR = LED_BRIGHTNESS;
uint16_t counterAutoTuneBrightness = 5;

HRSPO2Lib::HRSPO2Lib() {}

// 
boolean HRSPO2Lib::begin(void) {

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    return false;
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = LED_BRIGHTNESS;  //Options: 0=Off to 255=50mA
  byte ledMode = LED_MODE;              //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = SAMPLING_RATE;       //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  byte sampleAverage = SAMPLE_AVERAGE;  //Options: 1, 2, 4, 8, 16, 32
  int pulseWidth = PULSE_WIDTH;         //Options: 69, 118, 215, 411
  int adcRange = ADC_RANGE;             //Options: 2048, 4096, 8192, 16384

  particleSensor.softReset();
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  return true;
}

//
void HRSPO2Lib::run(void) {

  int32_t irValue = particleSensor.getIR();
  int32_t redValue = particleSensor.getRed();

  Serial.println(irValue);

  if (irValue > 5000) {
    dcFilterIR = dcRemovalFloat((float)irValue, dcFilterIR.w, ALPHA);
    float lpbFilterIR = lowPassButterworthFilter(dcFilterIR.y, BUTTERWORTH_FILTER_TYPE);

    if (checkForBeat(lpbFilterIR) == true) {
      beat = !beat;
      samplesDelay = millis();
      // last_HRSpO2 = current_HRSpO2;

      //We sensed a beat!
      long delta = millis() - lastBeatTime;
      lastBeatTime = millis();

      current_HRSpO2.beatsPerMinute = 60 / (delta / 1000.0);

      // current_HRSpO2.beatAvg = mediaMovelFilter(current_HRSpO2.beatsPerMinute);
      current_HRSpO2.beatAvg = mediaBPMFilter(current_HRSpO2.beatsPerMinute);

      if ( ( current_HRSpO2.beatsPerMinute > ( current_HRSpO2.beatAvg - DIFF_MEASSURE_BEAT * current_HRSpO2.beatAvg ) ) && 
           ( current_HRSpO2.beatsPerMinute < ( current_HRSpO2.beatAvg + DIFF_MEASSURE_BEAT * current_HRSpO2.beatAvg ) ) ) {
        newValidSample = true;
        current_HRSpO2.validSample = true;
        counterAutoTuneBrightness = 5;
        counterDelay = SAMPLES_FAULT_COUNTER;
      }
    }
  }

  if ( millis() > ( samplesDelay + HRSPO2_SAMPLE_DELAY_MS ) ) {
    samplesDelay = millis();


    if ( (!counterDelay) && ( !counterAutoTuneBrightness-- ) ) {
      counterAutoTuneBrightness = 3;
      autoTuneBrightnessIR();
    }

    if ( !counterDelay-- ) {
      counterDelay = SAMPLES_FAULT_COUNTER;
      current_HRSpO2.beatAvg = 0;
      current_HRSpO2.validSample = false;
    }

    newValidSample = true;
  }

  if ( current_HRSpO2.beatAvg < 20 || current_HRSpO2.beatAvg > 220 ) {
    // newValidSample = false;
    current_HRSpO2.beatAvg = 0;
  }

  current_HRSpO2.temperature = particleSensor.readTemperature();

}

boolean HRSPO2Lib::avaliable(void) {

  return newValidSample;
}

HRSPO2_t HRSPO2Lib::getHRSpO2(void) {
  HRSPO2_t sample = current_HRSpO2;
  current_HRSpO2.validSample = false;
  newValidSample = false;  
  return sample;
}

//
void HRSPO2Lib::autoTuneBrightnessIR(void) {
  brightnessIR += 5;
  setBrightnessIR(brightnessIR);

  Serial.print("LED_BRIGHTNESS: ");
  Serial.println(brightnessIR);
}

// 
void HRSPO2Lib::setBrightnessIR(uint8_t value) {
  brightnessIR = value;
  particleSensor.setPulseAmplitudeIR(brightnessIR);
}

// Filter IIR
// x: current input sample,
// w: intermediate value that acts like a history of the DC value of the signal, 
// alpha: scale factor that widens or narrows the filter
dcFilter_t HRSPO2Lib::dcRemovalFloat(float x, float w, float alpha) {
  dcFilter_t filtered;
  filtered.w = x + alpha*w;
  filtered.y = filtered.w - w;
  return filtered;
}

//
float HRSPO2Lib::lowPassButterworthFilter(float x, uint8_t type) {
  valueBwF[0] = valueBwF[1];

  switch (type) {
  case 0:
    //Fs = 200Hz and Fc = 8Hz
    valueBwF[1] = (1.121602444751934047e-1 * x) + (0.77567951104961319064 * valueBwF[0]); //Very precise butterworth filter
    break;
  
  case 1:
    //Fs = 200Hz and Fc = 10Hz
    valueBwF[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * valueBwF[0]); //Very precise butterworth filter
    break;

  default:
    //Fs = 100Hz and Fc = 10Hz
    valueBwF[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * valueBwF[0]); //Very precise butterworth filter
    break;
  }

  return (valueBwF[0] + valueBwF[1]);
}

//
uint16_t HRSPO2Lib::mediaMovelFilter(float sample) {
  static float media = 0.0;
  static int indice = 1;

  if ( indice == 0 || indice == MEDIA_MOVEL_SAMPLE_SIZE ) {
    indice = 1;
    media = 0.0;
  }

  media = media + ( sample - media ) / indice++;

  return (uint16_t)media;
}

//
uint16_t HRSPO2Lib::mediaBPMFilter(float sampleBPM) {
  uint16_t beatAvg;
  if (sampleBPM < SAMPLE_BPM_MAX && sampleBPM > SAMPLE_BPM_MIN) {
      rates[rateSpot++] = (uint16_t)sampleBPM; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  return beatAvg;
}

//
boolean HRSPO2Lib::checkForBeat(float sample) {
  bool beatDetected = false;
  IR_AC_Signal_Previous = IR_AC_Signal_Current;
  IR_AC_Signal_Current = sample;

  //  Detect positive zero crossing (rising edge)
  if ( ( IR_AC_Signal_Previous < 0 ) & ( IR_AC_Signal_Current >= 0 ) ) {
    IR_AC_Max = IR_AC_Signal_Max;
    IR_AC_Min = IR_AC_Signal_Min;

    positive_Edge = 1;
    negative_Edge = 0;
    IR_AC_Signal_Max = 0;

    if ((IR_AC_Max - IR_AC_Min) > IR_AC_MINIMO_DELTA & (IR_AC_Max - IR_AC_Min) < IR_AC_MAXIMO_DELTA)
    {
      beatDetected = true;    //Heart beat!!!
    }

  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0))
  {
    positive_Edge = 0;
    negative_Edge = 1;
    IR_AC_Signal_Min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positive_Edge & (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_Max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negative_Edge & (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_Min = IR_AC_Signal_Current;
  }

  return beatDetected;
}