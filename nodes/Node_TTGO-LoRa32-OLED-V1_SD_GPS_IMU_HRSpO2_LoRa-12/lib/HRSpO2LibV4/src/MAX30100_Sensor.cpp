#include <Arduino.h>

#include "MAX30100_Sensor.h"

PulseOximeter sensor;
void onBeatDetected();

HRSpO2::HRSpO2() {}

// Callback (registered below) fired when a pulse is detected
void onBeatDetected() {
    // Serial.println("Beat!");
}

bool HRSpO2::begin(void) {

    // Inicializacao do sensor HR & SpO2
    // Options:
    //  * PULSEOXIMETER_DEBUGGINGMODE_NONE : 
    //  * PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT : filtered samples and beat detection threshold
    //  * PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES : sampled values coming from the sensor, with no processing
    //  * PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES : sampled values after the DC removal filter
    if (!sensor.begin(PULSEOXIMETER_DEBUGGINGMODE_NONE)) {
    // if (!sensor.begin(PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT)) {
        Serial.println("HRSpO2 sensor initialized with FAULT");
        return false;
    } else {
        Serial.println("HRSpO2 sensor initialized with SUCCESS");

        // sensor.setIRLedCurrent(MAX30100_LED_CURR_0MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_11MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_14_2MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_11MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_17_4MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_20_8MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_24MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_27_1MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_30_6MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_33_8MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_37MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_40_2MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_43_6MA);
        // sensor.setIRLedCurrent(MAX30100_LED_CURR_46_8MA);
        // Register a callback for the beat detection
        sensor.setOnBeatDetectedCallback(onBeatDetected);
        return true;
    }
}

void HRSpO2::run(void) {
    sensor.update();

    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

        paramHRSpO2.HR = sensor.getHeartRate();
        paramHRSpO2.SpO2 = sensor.getSpO2();
        paramHRSpO2.Temperature = sensor.getTemperature();

        newDatas = true;

        // if (DEBUG) {
        //     Serial.print("[");
        //     Serial.print(paramHRSpO2.HR);
        //     Serial.print(",");
        //     Serial.print(paramHRSpO2.SpO2);
        //     Serial.print(",");
        //     Serial.print(paramHRSpO2.Temperature);
        //     Serial.println("]");
        // }

        tsLastReport = millis();
    }

}

bool HRSpO2::avaliable(void) {
    bool status = newDatas;
    newDatas = false;
    return status;
}

String HRSpO2::getStrParamHRSpO2(uint64_t unixTime) {
    String result = "[";
    // resut += HRSPO2_TYPE;
    // resut += ",";
    result += (unsigned long)unixTime;
    result += ",";
    result += paramHRSpO2.HR;
    result += ",";
    result += paramHRSpO2.SpO2;
    result += ",";
    result += paramHRSpO2.Temperature;
    result += ",";
    result += getVBat();
    result += "]";
    return result;
}

// Mede a diferenca de potencial (Vpp) da bateria
// Inputs: nenhum
// Return: float com o valor atual Vpp da bateria
float HRSpO2::getVBat(void) {
    return (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;;
}
