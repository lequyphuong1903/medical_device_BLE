#include <Arduino.h>
#include "hrSensor.h"
#include "button.h"
#include "BLE.h"

BLE ble;                          // Bluetooth Low Energy
HeartRateSensor heartRateSensor;  // Sensor
Button button;                    // Button

void setup() {
  Serial.begin(115200);
  heartRateSensor.begin_sensor();
  heartRateSensor.setup_sensor();
  ble.initBLE();
  button.setup();
  delay(20);                      // Prevent button debounce
}

void loop() {
  if (ble.pServer->getConnectedCount() > 0) {
    heartRateSensor.check_fifo();
    while (heartRateSensor.sensor_available()) //do we have new data?
    {
      heartRateSensor.sensor_read();
      heartRateSensor.sensor_tranmit();
      ble.pCharacteristic->setValue(heartRateSensor.data,sizeof(heartRateSensor.data));
      ble.pCharacteristic->notify();
    }
  }
  
  if(!ble._BLEClientConnected)
  {
    ble.reconnect();  
  }
  button.update(heartRateSensor);
}

