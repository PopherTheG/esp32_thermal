#ifndef COMPONENTS_TELEMETRY_TELEMETRY_PROTOCOL
#define COMPONENTS_TELEMETRY_TELEMETRY_PROTOCOL

#include <stdint.h>

enum {
    TELEMETRY_TYPE_AUTH = 0x01,
    TELEMETRY_TYPE_LOG  = 0x02,
};

enum {
    TELEMETRY_DEVICE_BLE = 0x01,
    TELEMETRY_DEVICE_BSCAN = 0x02,
};

typedef struct {
    uint8_t type;
    uint16_t version;
    uint8_t device_type;
    uint16_t reqNo;
    uint64_t serial;
    float temperature;
    uint8_t namespaceID[10];
    uint8_t instanceID[6];
} __attribute__((packed)) telemetry_t;

typedef struct {
    uint8_t type;    
    uint8_t device_type;
    uint16_t reqNo;
    uint64_t serial;
    uint16_t temperature;
    char uuid[37];
} __attribute__((packed)) telemetry_simple_t;

#endif /* COMPONENTS_TELEMETRY_TELEMETRY_PROTOCOL */
