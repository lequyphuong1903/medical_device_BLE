#include <Arduino.h>
#include "Wire.h"
#include "MAX30105.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define BUTTON_PIN_BITMASK 0x200000000
RTC_DATA_ATTR int bootCount = 0;
const int LONG_PRESS_TIME  = 1000;
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
bool isPressing = false;
bool isLongDetected = false;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUFFER_LENGTH 10

// Functions
void initSensor();
void initBLE();
void button();

// BLE
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;
bool _BLEClientConnected = false;
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};

// Sensor
byte device_id = 2;
MAX30105 particleSensor;
uint16_t irBuffer[BUFFER_LENGTH];
uint16_t redBuffer[BUFFER_LENGTH];
uint8_t data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(byte)];

void setup() {
  Serial.begin(115200);
  pinMode(GPIO_NUM_33, INPUT_PULLUP);
  initSensor();
  initBLE();
  ++bootCount;
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  Serial.flush();
}

void loop() {
  if (pServer->getConnectedCount() > 0) {
    particleSensor.check();
    while (particleSensor.available()) //do we have new data?
    {
      memcpy(data, &device_id, sizeof(byte));

      for(uint8_t i = 0; i < BUFFER_LENGTH; i++)
      {
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
      }

      memcpy(data + sizeof(byte), (byte*)redBuffer, sizeof(redBuffer));
      memcpy(data + sizeof(redBuffer) + sizeof(byte), (byte*)irBuffer, sizeof(irBuffer));
      pCharacteristic->setValue(data,sizeof(data));
      pCharacteristic->notify();
    }
  }
  button();
}
void initSensor()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  byte ledBrightness = 0x0F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableAFULL();
  particleSensor.setFIFOAlmostFull(3);
}
void initBLE()
{
  BLEDevice::init("IUD 2008 BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}
void button()
{
  currentState = digitalRead(GPIO_NUM_33);

  if(lastState == HIGH && currentState == LOW) {        // button is pressed
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  } else if(lastState == LOW && currentState == HIGH) { // button is released
    isPressing = false;
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      isLongDetected = true;
      esp_deep_sleep_start();
    }
  }

  // save the the last state
  lastState = currentState;
}