#include "button.h"

bool Button::update()
{
    currentState = digitalRead(WAKEUP_BUTTON);

    if(lastState == HIGH && currentState == LOW) {        // button is pressed
        pressedTime = millis();
        isPressing = true;
        isLongDetected = false;
    } else if(lastState == LOW && currentState == HIGH) { // button is released
        isPressing = false;
    }

    if(isPressing == true && isLongDetected == false) {
        long pressDuration = millis() - pressedTime;

        if(pressDuration > LONG_PRESS_TIME)
        {
            isLongDetected = true;
        }
    }
    lastState = currentState;
    return isLongDetected;
}

void Button::setup()
{
    int lastState = LOW;  // the previous state from the input pin
    int currentState;     // the current reading from the input pin
    unsigned long pressedTime  = 0;
    bool isPressing = false;
    bool isLongDetected = false;
    ++bootCount;
    pinMode(WAKEUP_BUTTON, INPUT_PULLUP);
    esp_sleep_enable_ext0_wakeup(WAKEUP_BUTTON,1);
    Serial.flush();
}