#include "BLE.h"

void BLE::MyServerCallbacks::onConnect(BLEServer *pServer, BLE& ble)
{
    ble._BLEClientConnected = true;
}

void BLE::MyServerCallbacks::onDisconnect(BLEServer *pServer, BLE& ble)
{
    ble._BLEClientConnected = false;
}

void BLE::initBLE()
{
    BLEDevice::init("IUD 2008 BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks);
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
void BLE::reconnect()
{
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}