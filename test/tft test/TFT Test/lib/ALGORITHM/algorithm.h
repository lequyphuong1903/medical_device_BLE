#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#define NSLOT       100
#define PADDING     2

class Pulse {
public:
    Pulse(void);
    void GetData(uint16_t data);
    void MovMean(uint16_t *buffer);
    void FindPeaks(uint16_t *buffer);
    void CalculatePara();
    int _spo2;
    int _hr;
    bool _isBeat;
    bool _preBeat;
    uint8_t _nextslot;
private:
    uint16_t buffer[NSLOT];
    uint16_t filBuffer[NSLOT-PADDING];
    int8_t peaks[10];
    int8_t _peakIndex;
    float temphr = 0.0f;
    int8_t preValue;
    int8_t curValue;
    int8_t numpeak;
    u_long preBeat;
    u_long curBeat;
};


#endif