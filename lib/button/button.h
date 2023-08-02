#ifndef BUTTON_H_
#define BUTTON_H_
#include <Arduino.h>
#include "hrSensor.h"

#define WAKEUP_BUTTON GPIO_NUM_33
#define BUTTON_PIN_BITMASK 0x200000000
const int LONG_PRESS_TIME = 1000;

class Button {
    public:
        void setup(void);
        bool update(void);
    private:
        int bootCount = 0;
        int lastState;
        int currentState;
        unsigned long pressedTime ;
        bool isPressing;
        bool isLongDetected;
};
#endif