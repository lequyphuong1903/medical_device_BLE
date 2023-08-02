#ifndef CALLBACK_SERVER_H_
#define CALLBACK_SERVER_H_
#include <Arduino.h>
#include <BLEServer.h>
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer);
    void onDisconnect(BLEServer *pServer);
    bool _BLEClientConnected = false;
};

#endif