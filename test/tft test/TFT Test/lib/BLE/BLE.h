#ifndef BLE_H_
#define BLE_H_
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"



class BLE
{
    public:
        void initBLE(void);
        void reconnect(void);
        BLEServer *pServer;
        BLEService *pService;
        BLECharacteristic *pCharacteristic;
        bool _BLEClientConnected = false;
        class MyServerCallbacks : public BLEServerCallbacks
        {
            private:
                void onConnect(BLEServer *pServer, BLE& ble);
                void onDisconnect(BLEServer *pServer, BLE& ble);
            public:
                
        };
    private:
        
};


#endif