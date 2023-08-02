#include "CallbackServer.h"

void MyServerCallbacks::onConnect(BLEServer *pServer)
{
    _BLEClientConnected = true;
}

void MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
    _BLEClientConnected = false;
}