#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <ESP32Time.h>
#include "Adafruit_SH1106.h"
#include "MAX30105.h"
#include "BLE.h"
#include "heartRate.h"

// TFT
#define OLED_MOSI         GPIO_NUM_23
#define OLED_CLK          GPIO_NUM_18
#define OLED_DC           GPIO_NUM_16
#define OLED_CS           GPIO_NUM_5
#define OLED_RESET        GPIO_NUM_17
Adafruit_SH1106 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#define SCALING 48                                          // Scale height of trace, reduce value to make trace height
#define TRACE_SPEED 0.08                                     // Speed of trace across screen, higher=faster   
#define TRACE_MIDDLE_Y_POSITION 41                          // y pos on screen of approx middle of trace
#define TRACE_HEIGHT 64                                     // Max height of trace in pixels    
#define HALF_TRACE_HEIGHT TRACE_HEIGHT/2                    // half Max height of trace in pixels (the trace amplitude)    
#define TRACE_MIN_Y TRACE_MIDDLE_Y_POSITION-HALF_TRACE_HEIGHT+1     // Min Y pos of trace, calculated from above values
#define TRACE_MAX_Y TRACE_MIDDLE_Y_POSITION+HALF_TRACE_HEIGHT-1     // Max Y pos of trace, calculated from above values
#if (SH1106_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SH1106.h!");
#endif


const unsigned char ble_ico [] PROGMEM = {
  B00000001, B11000000,
  B00000001, B10110000,
  B00000001, B10001100,
  B00000001, B10000011,
  B00110011, B10000011,
  B00001100, B10001100,
  B00000011, B10110000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B10110000,
  B00001101, B10001100,
  B00110001, B10000011,
  B00000001, B10000011,
  B00000001, B10001100,
  B00000001, B10110000,
  B00000001, B11000000,
};

const unsigned char non_ble_ico [] PROGMEM = {
  B10000001, B11000001,
  B01000001, B10110010,
  B00100001, B10001100,
  B00010001, B10001011,
  B00111011, B10010011,
  B00001100, B10101100,
  B00000011, B11110000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11110000,
  B00001101, B10101100,
  B00111001, B10010011,
  B00010001, B10001011,
  B00100001, B10001100,
  B01000001, B10110010,
  B10000001, B11000001,
};

const unsigned char lab_logo [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0x80, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xc0, 0x7f, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0x07, 0xff, 0xcf, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfc, 0x3f, 0xff, 0x8f, 0xf8, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf0, 0xff, 0xff, 0x07, 0xfe, 0x1f, 0xff, 0xff, 0xf8, 0x18, 0xc7, 0x1e, 0x1c, 0x03, 
	0xff, 0xff, 0xc3, 0xff, 0xfe, 0x07, 0xff, 0x8f, 0xff, 0xff, 0xf0, 0x18, 0x43, 0x08, 0x08, 0x03, 
	0xff, 0xff, 0x8f, 0xff, 0xfc, 0x03, 0xff, 0xc3, 0xff, 0xff, 0xe0, 0x18, 0xc3, 0x18, 0x08, 0x03, 
	0xff, 0xff, 0x1f, 0xff, 0xf8, 0x83, 0xff, 0xf1, 0xff, 0xff, 0xe0, 0x10, 0x80, 0x10, 0x08, 0x07, 
	0xff, 0xfe, 0x7f, 0xff, 0xf9, 0x81, 0xff, 0xf8, 0xff, 0xff, 0xe1, 0xf0, 0x80, 0x10, 0x8c, 0x7f, 
	0xff, 0xf8, 0xff, 0xff, 0xf1, 0x01, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0xf0, 0x80, 0x01, 0xf8, 0x0f, 
	0xff, 0xf1, 0xff, 0xff, 0xe3, 0x00, 0xff, 0xff, 0x3f, 0xff, 0xf0, 0xe1, 0x00, 0x21, 0xf0, 0x0f, 
	0xff, 0xf3, 0xff, 0xff, 0xc7, 0x10, 0xff, 0xff, 0x9f, 0xff, 0xf0, 0xe1, 0x00, 0x21, 0xf0, 0x0f, 
	0xff, 0xe7, 0xff, 0xff, 0x87, 0x10, 0x7f, 0xff, 0x8f, 0xff, 0xf8, 0x61, 0x00, 0x21, 0x10, 0x0f, 
	0xff, 0xcf, 0xff, 0xff, 0x0f, 0x30, 0x7f, 0xff, 0xc7, 0xff, 0xc8, 0x61, 0x10, 0x62, 0x10, 0xff, 
	0xff, 0x9f, 0xff, 0xfc, 0x3f, 0x30, 0x3f, 0xff, 0xe7, 0xff, 0x80, 0x42, 0x10, 0x60, 0x10, 0x3f, 
	0xff, 0x1f, 0xff, 0xf8, 0x3f, 0x30, 0x3f, 0xff, 0xf3, 0xff, 0x80, 0x42, 0x10, 0x60, 0x30, 0x1f, 
	0xff, 0x3f, 0xff, 0xf0, 0x7e, 0x70, 0x1f, 0xff, 0xf9, 0xff, 0x80, 0xc2, 0x18, 0x60, 0x70, 0x1f, 
	0xfe, 0x7f, 0xff, 0xe0, 0xfc, 0x70, 0x1f, 0xff, 0xf9, 0xff, 0x81, 0xc6, 0x38, 0xf0, 0xf0, 0x1f, 
	0xfe, 0x7f, 0xff, 0xc0, 0x9c, 0x70, 0x1f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0xff, 0xff, 0x81, 0x18, 0x60, 0x0f, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xfc, 0xff, 0xff, 0x82, 0x18, 0x00, 0x0f, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xf9, 0xff, 0xff, 0x00, 0x00, 0x00, 0x07, 0xff, 0xfe, 0x7f, 0xfc, 0x7f, 0x1f, 0xe7, 0xf8, 0xff, 
	0xf9, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x3f, 0xf0, 0x3e, 0x0f, 0x81, 0xe0, 0x3f, 
	0xf3, 0xff, 0xe0, 0x00, 0x00, 0x20, 0x01, 0xcf, 0xff, 0x3f, 0xe0, 0x18, 0x07, 0x01, 0xc0, 0x3f, 
	0xf3, 0xff, 0x83, 0x00, 0x00, 0x60, 0x00, 0x07, 0xff, 0x3f, 0xc0, 0x18, 0x06, 0x01, 0x80, 0x3f, 
	0xf3, 0xff, 0x0e, 0x00, 0x03, 0xe0, 0xc0, 0x03, 0xff, 0x9f, 0xc2, 0x10, 0x84, 0x11, 0x86, 0x7f, 
	0xe7, 0xfe, 0x3c, 0x00, 0x07, 0xc0, 0x80, 0x01, 0xff, 0x9f, 0xfc, 0x21, 0x84, 0x31, 0xc0, 0x7f, 
	0xe7, 0xfc, 0xf8, 0x00, 0x0f, 0xc1, 0x00, 0xf0, 0xff, 0x9f, 0xf8, 0x21, 0x80, 0x61, 0xc0, 0x7f, 
	0xe7, 0x01, 0xf0, 0x00, 0x0f, 0xc0, 0x0f, 0xf8, 0x7f, 0xcf, 0xf8, 0x63, 0x88, 0x61, 0x80, 0xff, 
	0xe7, 0x07, 0x60, 0x00, 0x1f, 0x80, 0x1f, 0xf0, 0x1f, 0xcf, 0xf0, 0xc3, 0x08, 0xe3, 0x81, 0xff, 
	0xe6, 0x1c, 0x40, 0x00, 0x3f, 0x80, 0x3f, 0xc0, 0x0f, 0xcf, 0xe1, 0xc3, 0x10, 0xc3, 0x01, 0xff, 
	0xec, 0x38, 0x00, 0x00, 0xff, 0x80, 0x7f, 0x00, 0x03, 0xcf, 0xc3, 0xc6, 0x10, 0x86, 0x11, 0xff, 
	0xc8, 0xf0, 0x00, 0x01, 0xff, 0x80, 0xff, 0x00, 0x01, 0xcf, 0x81, 0xc0, 0x30, 0x04, 0x21, 0xff, 
	0xc1, 0xe0, 0x00, 0x03, 0xff, 0x01, 0xff, 0x80, 0x00, 0x4f, 0x00, 0xc0, 0x70, 0x0c, 0x01, 0xff, 
	0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xc0, 0xf8, 0x1e, 0x03, 0xff, 
	0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x81, 0xe1, 0xfc, 0x3f, 0x07, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x83, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfc, 0x06, 0x03, 0xc0, 0x80, 0x0f, 0xff, 0x81, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xe1, 0xc0, 0x03, 0xff, 0xc3, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf1, 0xc1, 0xe0, 0xff, 0xc3, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xe0, 0xff, 0xc3, 0xff, 0xff, 0xc3, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xff, 0x03, 0xc2, 0x0f, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xfc, 0x01, 0xc0, 0x07, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xfc, 0x61, 0xc1, 0x87, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xfc, 0xf1, 0xc3, 0x83, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xff, 0xf1, 0xc3, 0xc3, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xff, 0x01, 0xc3, 0xc3, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xfc, 0x01, 0xc3, 0xc3, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x07, 0xf3, 0xc1, 0xf0, 0x7f, 0xc3, 0xc8, 0x71, 0xc3, 0xc3, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x87, 0xe3, 0xc1, 0xe0, 0xff, 0xc3, 0xc8, 0x61, 0xc3, 0xc7, 0xff, 0xff, 
	0xff, 0xff, 0xfe, 0x0f, 0x83, 0xe7, 0xc1, 0xc1, 0xff, 0xc3, 0x88, 0x61, 0xc1, 0x87, 0xff, 0xff, 
	0xff, 0xff, 0xfc, 0x07, 0xc0, 0x0f, 0x80, 0x03, 0xff, 0x80, 0x08, 0x00, 0xc0, 0x0f, 0xff, 0xff, 
	0xff, 0xff, 0xfc, 0x07, 0xe0, 0x1f, 0x80, 0x1f, 0xff, 0x80, 0x0e, 0x30, 0xe0, 0x1f, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// Button
#define BUTTON_MOVE       GPIO_NUM_2
#define BUTTON_SELECT     GPIO_NUM_33
const int PRESS_TIME_INTERVAL = 1000;
int lastState = LOW;
int currentState = LOW;
unsigned long pressedTimeMoved     = 0;
unsigned long pressedTimeSelected  = 0;
int buttonindex = 0;
int systemindex = 0;
int recordindex = 0;
int clockindex  = 0;
int countsleep  = 0;

void setmenu(int expression);
void setsystem(int expression);
void setrecord(int expression, bool mode);
void setclock(int expression, bool mode);

volatile boolean SELECT = false;
volatile boolean MOVE   = false;
volatile boolean SYSTEM = false;
volatile boolean CLOCK  = false;
volatile boolean RECORD = false;
volatile boolean _CLOCK  = false; 
boolean CHANGE_MOVE = false;
boolean CHANGE_SELECT = false;

// Sensor
MAX30105 particleSensor;
#define BUFFER_LENGTH 10
uint16_t redBuffer[BUFFER_LENGTH];
uint16_t irBuffer[BUFFER_LENGTH];
uint16_t preValue = 0;
uint16_t curValue = 0;
uint8_t tempInt = 0;
uint8_t tempFrac = 0;
boolean isBeat = false;
float n_spo2_rf, ratio, correl; // SPO2 value
int8_t ch_spo2_valid_rf;        // indicator to show if the SPO2 calculation is valid
int32_t n_heart_rate_rf;        // heart rate value
int8_t ch_hr_valid_rf;          // indicator to show if the heart rate calculation is valid
int arr_index = 0;

// RTC
ESP32Time rtc(3600);
RTC_DATA_ATTR bool RECORD_MODE = false;
RTC_DATA_ATTR bool SET_TIME_MODE = false;
RTC_DATA_ATTR bool BOOT = false;
struct tm timeinfo;

// Heart Rate Calculate
const byte RATE_SIZE = 10; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

void HRCalculate()
{
  if (checkForBeat(curValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

void temperature_read()
{
  particleSensor.writeRegister8(0x57, 0x21, 0x01);
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = particleSensor.readRegister8(0x57, 0x01);
    if ((response & 0x02) > 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
  tempInt = particleSensor.readRegister8(0x57, 0x1F);
  tempFrac = particleSensor.readRegister8(0x57, 0x20); //Causes the clearing of the DIE_TEMP_RDY interrupt
}

hw_timer_t* timer0 = NULL;        // Init timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {   
  // portENTER_CRITICAL_ISR(&timerMux); // Anti conflict
  // ble.pCharacteristic->setValue(data,sizeof(data));
  // ble.pCharacteristic->notify();
  // portEXIT_CRITICAL_ISR(&timerMux); // Exit timer
}

// External interrupt functions
void IRAM_ATTR changeMove()
{
  if (millis() - pressedTimeMoved >= 200)
  {
    MOVE = true;
    SELECT = false;
    CHANGE_MOVE = true;
    pressedTimeMoved = millis();
  }
}

void IRAM_ATTR changeSelect()
{
  if (millis() - pressedTimeSelected >= 200)
  {
    SELECT = true;
    if (!MOVE)
      CHANGE_SELECT = !CHANGE_SELECT;
    pressedTimeSelected = millis();
  }
}

BLE ble;
boolean BLE_CONNECT = false;
uint8_t data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(tempInt) + sizeof(tempFrac) + sizeof(beatAvg)];


void setup()   {         
  Serial.begin(9600);
  if (!BOOT)
  {
    rtc.setTime(30, 24, 15, 17, 1, 2021);
    BOOT = true;
  }
  
  pinMode(BUTTON_MOVE, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(BUTTON_SELECT, 1);
  attachInterrupt(BUTTON_MOVE, changeMove, FALLING);
  attachInterrupt(BUTTON_SELECT, changeSelect, FALLING);
  display.begin(SH1106_SWITCHCAPVCC);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  ble.initBLE();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  byte ledBrightness = 0x2F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 8192; //Options: 2048, 4096, 8192, 16384
  particleSensor.setFIFOAverage(8);
  particleSensor.clearFIFO();
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableAFULL();
  particleSensor.setFIFOAlmostFull(3);

  // timer0 = timerBegin(0, 80, true);              // Init timer 0 with MCU Frequency 8MHz
  // timerAttachInterrupt(timer0, &onTimer, true);  // Init  interrupt function for Timer 0
  // timerAlarmWrite(timer0, 200000, true);          // Interrupt timer is (40000 us) 
  // timerAlarmEnable(timer0);                      // Timer 0 start

  display.drawBitmap(0, 0,  lab_logo, 128, 64, 1);
  display.invertDisplay(1);
  display.display();
  delay(2000);
  display.invertDisplay(0);
  display.clearDisplay();
  timeinfo = rtc.getTimeStruct();
  display.setCursor(50, 0);
  display.print(&timeinfo, "%H:%M");
  display.display();
}

void loop() {
  int16_t Diff = 0; 
  static float lastx = 1;                       // Last x position of trace
  static int lasty = TRACE_MIDDLE_Y_POSITION;   // Last y position of trace, default to middle
  static float x = 1;
  int32_t y;
  static int32_t SensorOffset = 10000;
  if(!MOVE && !CHANGE_SELECT)
  {
    timeinfo = rtc.getTimeStruct();
    particleSensor.check();
    while (particleSensor.available()) //do we have new data?
    {
      for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
      {
        redBuffer[i] = particleSensor.getFIFORed();
        irBuffer[i] = particleSensor.getFIFOIR();
        particleSensor.nextSample();
      }
    }
    curValue = irBuffer[0];
    HRCalculate();
    if (redBuffer[0] < 2000)
    {
      countsleep++;
      beatAvg = 0;
      beatsPerMinute = 0;
      tempInt = 0;
      tempFrac = 0;
      display.clearDisplay();
      display.setCursor(50,0);
      display.print(&timeinfo, "%H:%M");
      display.setCursor(33, 42);
      display.print("No Finger!");
      display.display();
      delay(1000);
      display.clearDisplay();
      display.setCursor(50,0);
      display.print(&timeinfo, "%H:%M");
      display.display();
      delay(1000);
      if (countsleep > 4)
      {
        display.drawBitmap(0, 0, lab_logo, 128, 64, 1);
        display.invertDisplay(1);
        display.display();
        delay(2000);
        display.invertDisplay(0);
        particleSensor.shutDown();
        esp_deep_sleep_start();
      }
    }
    else
    {
      countsleep = 0;
      Diff = (irBuffer[0] - redBuffer[0]); // Get raw difference between the 2 LEDS
      Diff = Diff - SensorOffset;    // Adjust the baseline of raw values by removing the offset (moves into a good range for scaling)
      Diff = Diff / SCALING; 
      if(Diff < -HALF_TRACE_HEIGHT)
        SensorOffset += (SCALING * (abs(Diff) - 32));
      if(Diff > HALF_TRACE_HEIGHT)
        SensorOffset += (SCALING * (abs(Diff) - 32));
        
      y = Diff + (TRACE_MIDDLE_Y_POSITION - HALF_TRACE_HEIGHT);  // These two lines move Y pos of trace to approx middle of display area
      y += TRACE_HEIGHT/4;

      if(y > TRACE_MAX_Y) y = TRACE_MAX_Y;                            // If going beyond trace box area then crop the trace
      if(y < TRACE_MIN_Y) y = TRACE_MIN_Y;                            // so it stays within
      display.drawLine(lastx, lasty, x, y, INVERSE);                  // Plot the next part of the trace
      lasty = y;                                                      // Save where the last Y pos was
      lastx = x;                                                      // Save where the last X pos was
      x += TRACE_SPEED;                                               // Move trace along the display

      if (x > 99)                                                     // If reached end of display then reset to statt
      {
        if(!ble._BLEClientConnected)
        {
          ble.reconnect();  
        }
        x = 1;                                                        // Back to start
        lastx = x;
        display.clearDisplay();
        temperature_read();
        display.setCursor(50,0);
        display.print(&timeinfo, "%H:%M");
        
        display.setCursor(104, 35);
        double temp = tempInt + 0.0625f*(double)tempFrac;
        display.print(temp,  1);  
        display.setCursor(104, 55);
        display.print(beatAvg);          
      }
      if(!BLE_CONNECT)
        display.drawBitmap(110, 0, non_ble_ico, 16, 16, 1);
      else
        display.drawBitmap(110, 0, ble_ico, 16, 16, 1);
      display.drawFastVLine(100, 9, 55, WHITE);
      display.setCursor(104, 25);
      display.println("Temp");
      display.setCursor(104, 45);
      display.println("HR");   
      display.display();
    }
    if (ble.pServer->getConnectedCount() > 0)
    {
      BLE_CONNECT = true;
      memcpy(data, &tempInt, sizeof(tempInt));
      memcpy(data + sizeof(tempInt), &tempFrac, sizeof(tempFrac));
      memcpy(data + sizeof(tempInt) + sizeof(tempFrac), (byte*)redBuffer, sizeof(redBuffer));
      memcpy(data + sizeof(redBuffer) + sizeof(tempInt) + sizeof(tempFrac), (byte*)irBuffer, sizeof(irBuffer));
      memcpy(data + sizeof(redBuffer) + sizeof(irBuffer) + sizeof(tempInt) + sizeof(tempFrac), &beatAvg, sizeof(beatAvg));
      ble.pCharacteristic->setValue(data,sizeof(data));
      ble.pCharacteristic->notify();
    }
    else
      BLE_CONNECT = false;
  }
  else if (CHANGE_SELECT && !MOVE && !SYSTEM && !_CLOCK && !RECORD)
  {
    if (millis() - pressedTimeSelected <= 10000)
    {
      display.clearDisplay();
      timeinfo = rtc.getTimeStruct();
      display.setCursor(18,10);
      display.print(&timeinfo, "%a, %b %d %Y");
      display.setTextSize(2);
      display.setCursor(35, 30);
      display.print(&timeinfo, "%H:%M");
      display.display();
      display.setTextSize(1);
    }
    else
    {
      display.clearDisplay();
      SELECT = false;
      CHANGE_SELECT = false;
      display.display();
    }
    
  }
  else
  {
    if (millis() - pressedTimeMoved <= 10000)
    {
      if (CHANGE_MOVE && !SYSTEM && !RECORD && !_CLOCK)
      {
        display.clearDisplay();
        setmenu(buttonindex);
        (++buttonindex) %= 4;
        CHANGE_MOVE = false;
      }
      else if (CHANGE_MOVE && SYSTEM && !RECORD && !_CLOCK)
      {
        display.clearDisplay();
        setsystem(systemindex);
        (++systemindex) %= 5;
        CHANGE_MOVE = false;
      }
      else if (CHANGE_MOVE && !SYSTEM && RECORD && !_CLOCK)
      {
        display.clearDisplay();
        setrecord(recordindex, RECORD_MODE);
        (++recordindex) %= 4;
        CHANGE_MOVE = false;
      }
      else if (CHANGE_MOVE && !SYSTEM && !RECORD && _CLOCK)
      {
        display.clearDisplay();
        setclock(clockindex, SET_TIME_MODE);
        (++clockindex) %= 7;
        CHANGE_MOVE = false;
      }
    }
    else
    {
      display.clearDisplay();
      MOVE = false;
      CHANGE_MOVE = false;
      CHANGE_SELECT = false;
      SYSTEM = false;
      SELECT = false;
      RECORD = false;
      _CLOCK = false;
      recordindex = 0;
      systemindex = 0;
      clockindex  = 0;
      buttonindex = 0;
    }
    if (SELECT && buttonindex == 0)
    {
      display.clearDisplay();
      MOVE = false;
      CHANGE_MOVE = false;
      SELECT = false;
      CHANGE_SELECT = false;
      buttonindex = 0;
    }
    else if (SELECT && buttonindex == 1 && !RECORD)
    {
      display.clearDisplay();
      RECORD = true;
      SELECT = false;
      setrecord(recordindex++, RECORD_MODE);
    }
    else if (SELECT && recordindex == 0 && RECORD)
    {
      display.clearDisplay();
      RECORD = false;
      SELECT = false;
      setmenu(buttonindex-1); 
    }
    else if (SELECT && buttonindex == 2 && !_CLOCK)
    {
      display.clearDisplay();
      _CLOCK = true;
      SELECT = false;
      setclock(clockindex++, SET_TIME_MODE);
    }
    else if (SELECT && _CLOCK)
    {
      switch (clockindex)
      {
        case 0:
          display.clearDisplay();
          _CLOCK = false;
          SELECT = false;
          setmenu(buttonindex-1);
          break;
        case 1:
          display.clearDisplay();
          SELECT = false;
          SET_TIME_MODE = !SET_TIME_MODE;
          setclock(clockindex-1, SET_TIME_MODE);
          break;
        case 2:
          if(SET_TIME_MODE)
          {
            display.clearDisplay();
            SELECT = false;
            timeinfo.tm_year++;
            if (timeinfo.tm_year > 2030)
              timeinfo.tm_year = 2020;
            rtc.setTimeStruct(timeinfo);
            setclock(clockindex-1, SET_TIME_MODE);
          }
          break;
        case 3:
          if(SET_TIME_MODE)
          {
            display.clearDisplay();
            SELECT = false;
            timeinfo.tm_mon++;
            rtc.setTimeStruct(timeinfo);
            setclock(clockindex-1, SET_TIME_MODE);
          }
          break;
        case 4:
          if(SET_TIME_MODE)
          {
            display.clearDisplay();
            SELECT = false;
            timeinfo.tm_mday++;
            if (timeinfo.tm_mday>31)
              timeinfo.tm_mday = 1;
            rtc.setTimeStruct(timeinfo);
            setclock(clockindex-1, SET_TIME_MODE);
          }
          break;
        case 5:
          if (SET_TIME_MODE)
          {
            display.clearDisplay();
            SELECT = false;
            timeinfo.tm_hour++;
            if (timeinfo.tm_hour>23)
              timeinfo.tm_hour = 0;
            rtc.setTimeStruct(timeinfo);
            setclock(clockindex-1, SET_TIME_MODE);
          }
          break;
        default:
          if (SET_TIME_MODE)
          {
            display.clearDisplay();
            SELECT = false;
            timeinfo.tm_min++;
            if (timeinfo.tm_min>59)
              timeinfo.tm_min = 0;
            rtc.setTimeStruct(timeinfo);
            setclock(clockindex-1, SET_TIME_MODE);
          }
          break;
      }
    }
    
    else if (SELECT && buttonindex == 3 && !SYSTEM)
    {
      display.clearDisplay();
      SYSTEM = true;
      SELECT = false;
      setsystem(systemindex++);
    }
    else if (SELECT && systemindex == 0 && SYSTEM)
    {
      display.clearDisplay();
      SYSTEM = false;
      SELECT = false;
      setmenu(buttonindex-1); 
    }
  }
}

void setclock(int expression, bool mode)
{
  String name_device = "Name     IUD BLE 2008";
  String clock     = "Clock";
  String mode_on   = "Set time           ON";
  String mode_off  = "Set time          OFF";
  String set_year  = "Set year             ";
  String set_month = "Set month            ";
  String set_day   = "Set day              ";
  String set_hour  = "Set hour             ";
  String set_min   = "Set min              ";
  String exit      = "Exit";
  switch (expression)
  {
    case 0:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0,10);
      if (!mode)
        display.print(mode_off);
      else
        display.print(mode_on);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0, 20);
      display.print(set_year);
      display.setCursor(103,20);
      display.print(&timeinfo,"%Y");
      display.setCursor(0, 30);
      display.print(set_month);
      display.setCursor(110,30);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 40);
      display.print(set_day);
      display.setCursor(115,40);
      display.print(&timeinfo,"%d");
      display.setCursor(0, 50);
      display.print(set_hour);
      display.setCursor(115,50);
      display.print(&timeinfo,"%H");
      display.display();
      break;
    case 1:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 20);
      display.print(set_year);
      display.setCursor(103,20);
      display.print(&timeinfo,"%Y");
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0,10);
      if (!mode)
        display.print(mode_off);
      else
        display.print(mode_on);
      display.setCursor(0, 30);
      display.print(set_month);
      display.setCursor(110,30);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 40);
      display.print(set_day);
      display.setCursor(115,40);
      display.print(&timeinfo,"%d");
      display.setCursor(0, 50);
      display.print(set_hour);
      display.setCursor(115,50);
      display.print(&timeinfo,"%H");
      display.display();
      break;
    case 2:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 30);
      display.print(set_month);
      display.setCursor(110,30);
      display.print(&timeinfo,"%b");
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0,10);
      if (!mode)
        display.print(mode_off);
      else
        display.print(mode_on);
      display.setCursor(0, 20);
      display.print(set_year);
      display.setCursor(103,20);
      display.print(&timeinfo,"%Y");
      display.setCursor(0, 40);
      display.print(set_day);
      display.setCursor(115,40);
      display.print(&timeinfo,"%d");
      display.setCursor(0, 50);
      display.print(set_hour);
      display.setCursor(115,50);
      display.print(&timeinfo,"%H");
      display.display();
      break;
    case 3:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 40);
      display.print(set_day);
      display.setCursor(115,40);
      display.print(&timeinfo,"%d");
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0,10);
      if (!mode)
        display.print(mode_off);
      else
        display.print(mode_on);
      display.setCursor(0, 20);
      display.print(set_year);
      display.setCursor(103,20);
      display.print(&timeinfo,"%Y");
      display.setCursor(0, 30);
      display.print(set_month);
      display.setCursor(110,30);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 50);
      display.print(set_hour);
      display.setCursor(115,50);
      display.print(&timeinfo,"%H");
      display.display();
      break;
    case 4:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 50);
      display.print(set_hour);
      display.setCursor(115,50);
      display.print(&timeinfo,"%H");
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0,10);
      if (!mode)
        display.print(mode_off);
      else
        display.print(mode_on);
      display.setCursor(0, 20);
      display.print(set_year);
      display.setCursor(103,20);
      display.print(&timeinfo,"%Y");
      display.setCursor(0, 30);
      display.print(set_month);
      display.setCursor(110,30);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 40);
      display.print(set_day);
      display.setCursor(115,40);
      display.print(&timeinfo,"%d");
      display.display();
      break;
    case 5:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(0, 50);
      display.print(set_min);
      display.setCursor(115,50);
      display.print(&timeinfo,"%M");
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0, 10);
      display.print(set_year);
      display.setCursor(103,10);
      display.print(&timeinfo,"%Y");
      display.setCursor(0, 20);
      display.print(set_month);
      display.setCursor(110,20);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 30);
      display.print(set_day);
      display.setCursor(115,30);
      display.print(&timeinfo,"%d");
      display.setCursor(0, 40);
      display.print(set_hour);
      display.setCursor(115,40);
      display.print(&timeinfo,"%H");
      display.display();
      break;
    default:
      delay(20);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(45, 50);
      display.print(exit);
      display.setTextColor(WHITE, BLACK);
      display.setCursor(45,0);
      display.print(clock);
      display.setCursor(0, 10);
      display.print(set_month);
      display.setCursor(110,10);
      display.print(&timeinfo,"%b");
      display.setCursor(0, 20);
      display.print(set_day);
      display.setCursor(115,20);
      display.print(&timeinfo,"%d");
      display.setCursor(0, 30);
      display.print(set_hour);
      display.setCursor(115,30);
      display.print(&timeinfo,"%H");
      display.setCursor(0, 40);
      display.print(set_min);
      display.setCursor(115,40);
      display.print(&timeinfo,"%M");
      display.display();
      break;
  }
}

void setrecord(int expression, bool mode)
{
  String record      = "Record";
  String mode_auto   = "Mode           Auto";
  String mode_manual = "Mode         Manual";
  String seg         = "Seg                ";
  String delete_all  = "Delete all         ";
  String exit        = "Exit";
  switch (expression)
  {
  case 0:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,10);
    if (!mode)
      display.print(mode_manual);
    else
      display.print(mode_auto);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(record);
    display.setCursor(0,20);
    display.print(seg);
    display.setCursor(0,30);
    display.print(delete_all);
    display.setCursor(40,50);
    display.print(exit);
    display.display();
    break;
  case 1:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,20);
    display.print(seg);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(record);
    display.setCursor(0,10);
    if (!mode)
      display.print(mode_manual);
    else
      display.print(mode_auto);
    display.setCursor(0,30);
    display.print(delete_all);
    display.setCursor(40,50);
    display.print(exit);
    display.display();
    break;
  case 2:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,30);
    display.print(delete_all);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(record);
    display.setCursor(0,10);
    if (!mode)
      display.print(mode_manual);
    else
      display.print(mode_auto);
    display.setCursor(0,20);
    display.print(seg);
    display.setCursor(40,50);
    display.print(exit);
    display.display();
    break;
  default:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(40,50);
    display.print(exit);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(record);
    display.setCursor(0,10);
    if (!mode)
      display.print(mode_manual);
    else
      display.print(mode_auto);
    display.setCursor(0,20);
    display.print(seg);
    display.setCursor(0,30);
    display.print(delete_all);
    display.display();
    break;
  }
}

void setsystem(int expression)
{
  String system      = "System";
  String hard_ver    = "Hard Version    2.0.0";
  String soft_ver    = "Soft Version    2.0.1";
  String ble_ver     = "Bluetooth       V.4.2";
  String name_device = "Name     IUD BLE 2008";
  String exit        = "Exit";
  switch (expression)
  {
  case 0:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,10);
    display.print(hard_ver);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(system);
    display.setCursor(0,20);
    display.print(soft_ver);
    display.setCursor(0,30);
    display.print(ble_ver);
    display.setCursor(0,40);
    display.print(name_device);
    display.setCursor(50,50);
    display.print(exit);
    display.display();
    break;
  case 1:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,20);
    display.print(soft_ver);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(system);
    display.setCursor(0,10);
    display.print(hard_ver);
    display.setCursor(0,30);
    display.print(ble_ver);
    display.setCursor(0,40);
    display.print(name_device);
    display.setCursor(50,50);
    display.print(exit);
    display.display();
    break;
  case 2:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,30);
    display.print(ble_ver);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(system);
    display.setCursor(0,10);
    display.print(hard_ver);
    display.setCursor(0,20);
    display.print(soft_ver);
    display.setCursor(0,40);
    display.print(name_device);
    display.setCursor(50,50);
    display.print(exit);
    display.display();
    break;
  case 3:
    display.setTextColor(BLACK, WHITE);
    display.setCursor(0,40);
    display.print(name_device);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(45,0);
    display.print(system);
    display.setCursor(0,10);
    display.print(hard_ver);
    display.setCursor(0,20);
    display.print(soft_ver);
    display.setCursor(0,30);
    display.print(ble_ver);
    display.setCursor(50,50);
    display.print(exit);
    display.display();
    break;
  default:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(50,50);
    display.print(exit);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0,10);
    display.print(hard_ver);
    display.setCursor(45,0);
    display.print(system);
    display.setCursor(0,20);
    display.print(soft_ver);
    display.setCursor(0,30);
    display.print(ble_ver);
    display.setCursor(0,40);
    display.print(name_device);
    display.display();
    break;
  }
}

void setmenu(int expression)
{
  String menu   = "Main Menu";
  String record = "Record        ";
  String clock  = "Clock         ";
  String system = "System        ";
  String exit   = "Exit";
  char cursor = 16;
  switch (expression)
  {
  case 0:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(15,10);
    display.print(record);
    display.setCursor(100,10);
    display.print(cursor);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(35,0);
    display.print(menu);
    display.setCursor(15,20);
    display.print(clock);
    display.setCursor(100,20);
    display.print(cursor);
    display.setCursor(15,30);
    display.print(system);
    display.setCursor(100,30);
    display.print(cursor);
    display.setCursor(50,40);
    display.print(exit);
    display.display();
    break;
  case 1:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(15,20);
    display.print(clock);
    display.setCursor(100,20);
    display.print(cursor);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(35,0);
    display.print(menu);
    display.setCursor(15,10);
    display.print(record);
    display.setCursor(100,10);
    display.print(cursor);
    display.setCursor(15,30);
    display.print(system);
    display.setCursor(100,30);
    display.print(cursor);
    display.setCursor(50,40);
    display.print(exit);
    display.display();
    break;
  case 2:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(15,30);
    display.print(system);
    display.setCursor(100,30);
    display.print(cursor);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(35,0);
    display.print(menu);
    display.setCursor(15,10);
    display.print(record);
    display.setCursor(100,10);
    display.print(cursor);
    display.setCursor(15,20);
    display.print(clock);
    display.setCursor(100,20);
    display.print(cursor);
    display.setCursor(50,40);
    display.print(exit);
    display.display();
    break;
  default:
    delay(20);
    display.setTextColor(BLACK, WHITE);
    display.setCursor(50,40);
    display.print(exit);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(35,0);
    display.print(menu);
    display.setCursor(15,10);
    display.print(record);
    display.setCursor(100,10);
    display.print(cursor);
    display.setCursor(15,20);
    display.print(clock);
    display.setCursor(100,20);
    display.print(cursor);
    display.setCursor(15,30);
    display.print(system);
    display.setCursor(100,30);
    display.print(cursor);
    display.display();
    break;
  }
}
