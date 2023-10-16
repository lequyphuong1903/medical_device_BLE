#include <Arduino.h>
#include "algorithm.h"

Pulse::Pulse(){
    _spo2 = 0;
    _hr = 0;
    _isBeat = false;
    _preBeat = false;
    _nextslot = 0;
    for (uint8_t i = 0; i < 10; i++)
    {
        peaks[i] = -1;
    }
    _peakIndex = 0;
    temphr = 0.0f;
    preValue = 0;
    curValue = 0;
    numpeak = 0;
    preBeat = 0;
    curBeat = 0;
}

void Pulse::GetData(uint16_t data)
{
    buffer[_nextslot] = data;
    _nextslot++;
}

uint32_t SumPadding(uint16_t *data)
{
    uint32_t sum = 0;
    for (uint i = 0; i < PADDING; i++)
        sum += data[i];
    return sum;
}

void Pulse::MovMean(uint16_t *data)
{
    uint32_t sum = SumPadding(data);
    uint32_t threshold = sum/PADDING;

    for (uint8_t i = PADDING; i < NSLOT; i++)
    {
        filBuffer[i - PADDING] = sum / PADDING;
        uint16_t test = filBuffer[i - PADDING];
        Serial.println(test);
        if (test < 1000)
        {
            _isBeat = true;
            preBeat = true;
            
        }
        else
        {
            if (preBeat == true)
            {
                //Serial.println("Beat!");
                // delay(100);
            }
            _isBeat = false;
            preBeat = false;
            
        }
        if (i < NSLOT)
            sum = sum - filBuffer[i - 20] + data[i];
        
    }
}

void Pulse::FindPeaks(uint16_t *buffer)
{
    int prev_sample = buffer[0];
    int current_sample;
    for (uint8_t i = 1; i < NSLOT - PADDING; i++)
    {
        current_sample = buffer[i];
        
        if (current_sample > prev_sample)
        {
            _isBeat = true;
        }
         else if (current_sample < prev_sample)
         {
            if (_isBeat == false)
            {
                peaks[_peakIndex] = i - 1;
                _peakIndex++;
                _isBeat = true;
            }
        }
        prev_sample = current_sample;
    }
    temphr = 0.0f;
    numpeak = 0;
    if (peaks[0] != -1)
    {
        preValue = peaks[0];
        curValue = 0;
        numpeak++;
        for (uint8_t i = 1; i < 10; i++)
        {
            if (peaks[i] == -1)
                break;
            else
            {
                curValue = peaks[i];
                temphr += (curValue - preValue);
                preValue = curValue;
                numpeak++;
            }
        }
        
    }
    // Serial.println(numpeak);
}

void Pulse::CalculatePara()
{
    if (_nextslot >= 100)
    {
        MovMean(buffer);
        //FindPeaks(filBuffer);
        _nextslot = 0;
        numpeak = 0;
    }
}