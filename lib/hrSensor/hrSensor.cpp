#include "hrSensor.h"

void HeartRateSensor::setup_sensor()
  {
    particleSensor.setup(Sensor_para[0], Sensor_para[1], Sensor_para[2], Sensor_para[3], Sensor_para[4], Sensor_para[5]);
    particleSensor.enableAFULL();
    particleSensor.setFIFOAlmostFull(3);
  }

void HeartRateSensor::begin_sensor()
{
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
      Serial.println("MAX30105 was not found. Please check wiring/power. ");
      while(1);
    }
}

void HeartRateSensor::config_sensor(uint16_t para_0, uint16_t para_1, uint16_t para_2, uint16_t para_3, uint16_t para_4, uint16_t para_5)
{
  Sensor_para[0] = para_0;
  Sensor_para[1] = para_1;
  Sensor_para[2] = para_2;
  Sensor_para[3] = para_3;
  Sensor_para[4] = para_4;
  Sensor_para[5] = para_5;
}

void HeartRateSensor::check_fifo()
{
  particleSensor.check();
}

bool HeartRateSensor::sensor_available()
{
  return particleSensor.available();
}

void HeartRateSensor::sensor_sleep()
{
  config_sensor(0, Sensor_para[1], Sensor_para[2], Sensor_para[3],Sensor_para[4], Sensor_para[5]);
  setup_sensor();
}

void HeartRateSensor::sensor_read()
{
  for(uint8_t i = 0; i < BUFFER_LENGTH; i++)
  {
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

void HeartRateSensor::sensor_tranmit()
{
  memcpy(data, &device_id, sizeof(byte));
  memcpy(data + sizeof(byte), (byte*)redBuffer, sizeof(redBuffer));
  memcpy(data + sizeof(redBuffer) + sizeof(byte), (byte*)irBuffer, sizeof(irBuffer));
}