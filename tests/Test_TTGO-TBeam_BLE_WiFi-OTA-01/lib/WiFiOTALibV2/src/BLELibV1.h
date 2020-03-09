#ifndef BLELIBV1_H
#define BLELIBV1_H

#include <Arduino.h>
#include "SimpleBLE.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// BLE Bluetooth
#define BLE_TASK_DELAY_MS               (1000)

class BLELib
{
    public:

        BLELib();     // construtor

        boolean begin(void);
        void print(String strValue);

    private:

};

#ifdef __cplusplus
}
#endif

#endif // BLELIBV1_H