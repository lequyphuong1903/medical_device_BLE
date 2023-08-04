#include "button.h"

void Button::update(HeartRateSensor cambien)
{
    currentState = digitalRead(WAKEUP_BUTTON);

    if(lastState == HIGH && currentState == LOW) {        // button is pressed
        digitalWrite(LED_BATTERY, HIGH);
        pressedTime = millis();
        isPressing = true;
        isLongDetected = false;
    } else if(lastState == LOW && currentState == HIGH) { // button is released
        isPressing = false;
        digitalWrite(LED_BATTERY, LOW);
    }

    if(isPressing == true && isLongDetected == false) {
        long pressDuration = millis() - pressedTime;

        if(pressDuration > LONG_PRESS_TIME)
        {
            isLongDetected = true;
            cambien.sensor_sleep();
            esp_deep_sleep_start();   
        }
    }
    lastState = currentState;
}

void Button::setup()
{
    ++bootCount;
    pinMode(WAKEUP_BUTTON, INPUT_PULLUP);
    pinMode(LED_BATTERY, OUTPUT);
    esp_sleep_enable_ext0_wakeup(WAKEUP_BUTTON,1);
    Serial.flush();
}