#ifndef COMPONENTS_BLUETOOTH_BLE_TELEMETRY
#define COMPONENTS_BLUETOOTH_BLE_TELEMETRY

#include <stdint.h>

typedef struct
{
    uint8_t type;
    uint16_t reqNo;
    uint64_t serial;
    uint8_t namespaceID[10];
    uint8_t instanceID[6];
} __attribute__((packed)) ble_telemetry_t;

#endif /* COMPONENTS_BLUETOOTH_BLE_TELEMETRY */
