#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

/* Filter parameters */
#define ALPHA 0.95  //dc filter alpha value
#define MEAN_FILTER_SIZE        15  
#define PULSE_MIN_THRESHOLD     100 //300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
#define PULSE_MAX_THRESHOLD     2000
#define PULSE_BPM_SAMPLE_SIZE   8 //Moving average size

typedef struct {
  float w;
  float y;
} dcFilter_t;

typedef struct {
  float v[2];
  float result;
} butterworthFilter_t;

typedef struct {
  float values[MEAN_FILTER_SIZE];
  byte index;
  float sum;
  byte count;
} meanDiffFilter_t;

/* Enums, data structures and typdefs. DO NOT EDIT */
typedef struct  {
  bool pulseDetected;
  float heartBPM;

  float irCardiogram;

  float irDcValue;

  uint32_t lastBeatThreshold;

  float dcFilteredIR;
} pulseoxymeter_t;

typedef enum PulseStateMachine {
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;

dcFilter_t dcRemoval(float x, float w, float alpha);
float meanDiff(float M, meanDiffFilter_t* filterValues);
void lowPassButterworthFilter( float x, butterworthFilter_t * filterResult );
boolean detectPulse(float sensor_value);

dcFilter_t dcFilterIR;
butterworthFilter_t lpbFilterIR;
meanDiffFilter_t meanDiffIR;
uint8_t currentPulseDetectorState;
uint32_t lastBeatThreshold;
bool debug;
float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
uint8_t bpmIndex;
float valuesBPMSum;
uint8_t valuesBPMCount;
float currentBPM;
pulseoxymeter_t result;
uint16_t samplesRecorded;
uint16_t pulsesDetected;

float redACValueSqSum;
float irACValueSqSum;

byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

MAX30105 particleSensor;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  // if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 100kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 20;    //0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 0, 1 and 4 - 6 = Do not use; 2 = Heart Rate mode (Red only); 3 = SpO2 mode (Red + IR); 7 = Multi-LED mode (Red and IR)
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  //Arduino plotter auto-scales annoyingly. To get around this, pre-populate
  //the plotter with 500 of an average reading from the sensor

  //Take an average of IR readings at power up
  const byte avgAmount = 64;
  long baseValue = 0;
  for (byte x = 0 ; x < avgAmount ; x++)
  {
    baseValue += particleSensor.getIR(); //Read the IR value
  }
  baseValue /= avgAmount;

}

void loop()
{
  int32_t irValue = particleSensor.getIR();

  if (irValue > 5000) {
    dcFilterIR = dcRemoval((float)irValue, dcFilterIR.w, ALPHA);
    float meanDiffResIR = meanDiff( dcFilterIR.y, &meanDiffIR);
    lowPassButterworthFilter( meanDiffResIR, &lpbFilterIR );

    String strDatas = String(lpbFilterIR.result);
    strDatas += ',';
    strDatas += String(meanDiffResIR);
    strDatas += ',';
    strDatas += String(beatsPerMinute);

    Serial.println(strDatas);
  }


  if( detectPulse( lpbFilterIR.result ) && samplesRecorded > 0 )
  {
    result.pulseDetected=true;
    pulsesDetected++;

    float ratioRMS = log( sqrt(redACValueSqSum/samplesRecorded) ) / log( sqrt(irACValueSqSum/samplesRecorded) );

    if( debug == true )
    {
      Serial.print("RMS Ratio: ");
      Serial.println(ratioRMS);
    }

    //This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
    currentSaO2Value = 110.0 - 18.0 * ratioRMS;
    result.SaO2 = currentSaO2Value;
    
    if( pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
    {
      irACValueSqSum = 0;
      redACValueSqSum = 0;
      samplesRecorded = 0;
    }
  }

}

dcFilter_t dcRemoval(float x, float w, float alpha) {
  dcFilter_t filtered;
  filtered.w = x + alpha*w;
  filtered.y = filtered.w - w;
  return filtered;
}

float meanDiff(float M, meanDiffFilter_t* filterValues) {
  float avg = 0;

  filterValues->sum -= filterValues->values[filterValues->index];
  filterValues->values[filterValues->index] = M;
  filterValues->sum += filterValues->values[filterValues->index];

  filterValues->index++;
  filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

  if(filterValues->count < MEAN_FILTER_SIZE)
    filterValues->count++;

  avg = filterValues->sum / filterValues->count;
  return avg - M;
}

void lowPassButterworthFilter( float x, butterworthFilter_t * filterResult ) {  
  filterResult->v[0] = filterResult->v[1];

  //Fs = 100Hz and Fc = 10Hz
  filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

  //Fs = 100Hz and Fc = 4Hz
  //filterResult->v[1] = (1.367287359973195227e-1 * x) + (0.72654252800536101020 * filterResult->v[0]); //Very precise butterworth filter 

  filterResult->result = filterResult->v[0] + filterResult->v[1];
}

boolean detectPulse(float sensor_value) {
  static float prev_sensor_value = 0;
  static uint8_t values_went_down = 0;
  static uint32_t currentBeat = 0;
  static uint32_t lastBeat = 0;

  if(sensor_value > PULSE_MAX_THRESHOLD) {
    currentPulseDetectorState = PULSE_IDLE;
    prev_sensor_value = 0;
    lastBeat = 0;
    currentBeat = 0;
    values_went_down = 0;
    lastBeatThreshold = 0;
    return false;
  }

  switch(currentPulseDetectorState)
  {
    case PULSE_IDLE:
      if(sensor_value >= PULSE_MIN_THRESHOLD) {
        currentPulseDetectorState = PULSE_TRACE_UP;
        values_went_down = 0;
      }
      break;

    case PULSE_TRACE_UP:
      if(sensor_value > prev_sensor_value)
      {
        currentBeat = millis();
        lastBeatThreshold = sensor_value;
      }
      else
      {

        if(debug == true) 
        {
          Serial.print("Peak reached: ");
          Serial.print(sensor_value);
          Serial.print(" ");
          Serial.println(prev_sensor_value);
        }

        uint32_t beatDuration = currentBeat - lastBeat;
        lastBeat = currentBeat;

        float rawBPM = 0;
        if(beatDuration > 0)
          rawBPM = 60000.0 / (float)beatDuration;
        if(debug == true) 
          Serial.println(rawBPM);

        //This method sometimes glitches, it's better to go through whole moving average everytime
        //IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
        //valuesBPMSum -= valuesBPM[bpmIndex];
        //valuesBPM[bpmIndex] = rawBPM;
        //valuesBPMSum += valuesBPM[bpmIndex];

        valuesBPM[bpmIndex] = rawBPM;
        valuesBPMSum = 0;
        for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
        {
          valuesBPMSum += valuesBPM[i];
        }

        if(debug == true) 
        {
          Serial.print("CurrentMoving Avg: ");
          for(int i=0; i<PULSE_BPM_SAMPLE_SIZE; i++)
          {
            Serial.print(valuesBPM[i]);
            Serial.print(" ");
          }
  
          Serial.println(" ");
        }

        bpmIndex++;
        bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

        if(valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
          valuesBPMCount++;

        currentBPM = valuesBPMSum / valuesBPMCount;
        if(debug == true) 
        {
          Serial.print("AVg. BPM: ");
          Serial.println(currentBPM);
        }


        currentPulseDetectorState = PULSE_TRACE_DOWN;

        return true;
      }
      break;

    case PULSE_TRACE_DOWN:
      if(sensor_value < prev_sensor_value)
      {
        values_went_down++;
      }


      if(sensor_value < PULSE_MIN_THRESHOLD)
      {
        currentPulseDetectorState = PULSE_IDLE;
      }
      break;
  }

  prev_sensor_value = sensor_value;
  return false;
}