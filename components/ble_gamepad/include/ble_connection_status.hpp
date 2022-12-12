#pragma once

#include <cstdint>

#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)

#include "nimconfig.h"
#if defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL)

#include <NimBLEServer.h>
#include "NimBLECharacteristic.h"

class BleConnectionStatus : public NimBLEServerCallbacks {
public:
    BleConnectionStatus(void);
    ~BleConnectionStatus(void);

    virtual void onConnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo) override;
    virtual void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo& connInfo, int reason) override;

    bool connected = false;
    NimBLECharacteristic *inputGamepad;
};

#endif // CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#endif // CONFIG_BT_ENABLED
