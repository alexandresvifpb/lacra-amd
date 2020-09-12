#ifndef HRSPO2LIBV24_H
#define HRSPO2LIBV24_H

#include <Wire.h>
#include "MAX30105.h"
#include "LinkedList.h"

#ifdef __cplusplus
extern "C" {
#endif

// define o tipo do modulo
#define HRSPO2_TYPE                     (7)         // define como sendo modulo do tipo 2 (MPU9255)

#define HRSPO2_MAX301020_ADDRESS        (0x57)      
#define HRSPO2_PIN_SDA                  (21)        //
#define HRSPO2_PIN_SCL                  (22)        //

#define HRSPO2_TASK_DELAY_MS            (200)
#define HRSPO2_SAMPLE_DELAY_MS          (1000)
#define SAMPLES_FAULT_COUNTER           (10)

#define ALPHA                           (0.95)      //dc filter alpha value
#define IR_AC_MAXIMO_DELTA              (1000.00)
#define IR_AC_MINIMO_DELTA              (20.00)
#define IR_AC_INIT                      (20.00)

#define LED_BRIGHTNESS                  (30)        //Options: 0=Off to 255=50mA
#define LED_MODE                        (2)         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green

#define SAMPLING_RATE                   (100)       //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
#define SAMPLE_AVERAGE                  (1)         //Options: 1, 2, 4, 8, 16, 32
#define PULSE_WIDTH                     (400)       //Options: 69, 118, 215, 411
#define ADC_RANGE                       (8192)       //Options: 2048, 4096, 8192, 16384

// Butterworth Filter
#define BUTTERWORTH_FILTER_TYPE         (0)

// Media Beat
#define DIFF_MEASSURE_BEAT              (0.40)    // difference for the last measure in percentage
#define MEDIA_MOVEL_SAMPLE_SIZE         (30)

// mediaBPMFilter()
#define RATE_SIZE                       (4)
#define SAMPLE_BPM_MIN                  (20.0)
#define SAMPLE_BPM_MAX                  (255.0)

typedef struct {
  float y;          // output of the filter
  float w;          // intermediate value that acts like a history of the DC value of the signal
} dcFilter_t;

typedef struct {
  uint16_t beatsPerMinute;
  uint16_t beatAvg;
  float spo2;
  float spo2Avg;
  float temperature;
  boolean validSample;
} HRSPO2_t;

class HRSPO2Lib {
  public:

    HRSPO2Lib();

    boolean begin(void);
    void run(void);
    boolean avaliable(void);
    HRSPO2_t getHRSpO2(void);
    void autoTuneBrightnessIR(void);

    void setBrightnessIR(uint8_t value);

    uint32_t tsHRSPO2TaskDelayMS = HRSPO2_TASK_DELAY_MS;

  private:
    dcFilter_t dcRemovalFloat(float x, float w, float alpha);
    float lowPassButterworthFilter(float x, uint8_t type);
    uint16_t mediaMovelFilter(float sample);
    uint16_t mediaBPMFilter(float sampleBPM);

    boolean checkForBeat(float sample);

};

#ifdef __cplusplus
}
#endif

#endif /* HRSPO2LIBV24_H */