#ifndef BUTTON_H_
#define BUTTON_H_
#include <Arduino.h>
#include "hrSensor.h"

#define WAKEUP_BUTTON GPIO_NUM_33
#define LED_BATTERY GPIO_NUM_25
#define BUTTON_PIN_BITMASK 0x200000000
const int LONG_PRESS_TIME = 1000;

class Button {
    public:
        void setup(void);
        void update(HeartRateSensor cambien);
    private:
        int bootCount = 0;
        int lastState = LOW;  // the previous state from the input pin
        int currentState;     // the current reading from the input pin
        unsigned long pressedTime  = 0;
        bool isPressing = false;
        bool isLongDetected = false;
};
#endif