#ifndef COMPONENTS_TELEMETRY_TELEMETRY
#define COMPONENTS_TELEMETRY_TELEMETRY

#include <stdint.h>

typedef enum {
    TELEMETRY_ERR = -1,
    TELEMETRY_OK = 0,
}telemetry_err_t;

typedef enum
{
    TELEMETRY_TASK_STOP = -1,
    TELEMETRY_TASK_LOG,
} telemetry_msg_id_t;

typedef struct
{
    telemetry_msg_id_t id;
    uint8_t data;
} telemtery_msg_t;

void telemetry_init(void);

void telemetry_deinit(void);

void telemetry_start(uint64_t* id);

void telemetry_stop(void);

telemetry_err_t telemetry_notify_log(uint8_t device_type);

#endif /* COMPONENTS_TELEMETRY_TELEMETRY */
