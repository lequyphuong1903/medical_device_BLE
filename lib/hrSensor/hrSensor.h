#ifndef HRSENSOR_H_
#define HRSENSOR_H_

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

#define BUFFER_LENGTH 10

class HeartRateSensor {
private:
    MAX30105 particleSensor; // Create an instance of MAX30105

public:
    HeartRateSensor()
    {
        Sensor_para[0] = 0x0F; //Options: 0=Off to 255=50mA
        Sensor_para[1] = 1; //Options: 1, 2, 4, 8, 16, 32
        Sensor_para[2] = 2; //Options: 1, 2, 3
        Sensor_para[3] = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
        Sensor_para[4] = 411; //Options: 69, 118, 215, 411
        Sensor_para[5] = 8192; //Options: 2048, 4096, 8192, 16384
    }

    byte device_id = 2;
    uint16_t irBuffer[BUFFER_LENGTH];
    uint16_t redBuffer[BUFFER_LENGTH];
    uint8_t data[sizeof(irBuffer) + sizeof(redBuffer) + sizeof(byte)];
    uint16_t Sensor_para[6];

    void begin_sensor();
    void config_sensor(uint16_t para_0, uint16_t para_1, uint16_t para_2, uint16_t para_3, uint16_t para_4, uint16_t para_5);
    void setup_sensor();
    void check_fifo();
    bool sensor_available();
    void sensor_sleep();
    void sensor_read();
    void sensor_tranmit();
};
#endif