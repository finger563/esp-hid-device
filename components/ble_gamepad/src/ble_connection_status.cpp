#include "ble_connection_status.hpp"

BleConnectionStatus::BleConnectionStatus(void)
{
}

BleConnectionStatus::~BleConnectionStatus(void)
{
}

void BleConnectionStatus::onConnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo) {
    printf("connected");
    pServer->updateConnParams(connInfo.getConnHandle(), 6, 7, 0, 600);
    this->connected = true;
}

void BleConnectionStatus::onDisconnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo, int reason) {
    printf("disconnected\n");
    this->connected = false;
}
