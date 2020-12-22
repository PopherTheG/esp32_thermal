#ifndef COMPONENTS_SCANNER_SCANNER_APP
#define COMPONENTS_SCANNER_SCANNER_APP

#include <stdint.h>
#include <stdlib.h>



typedef enum {
    SCANNER_STATUS_ERR = -1,
    SCANNER_STATUS_OK   = 0,
} scanner_status_t;

typedef enum {
    SCANNER_EVT_FAIL,
    SCANNER_EVT_UUID_VALID,
    SCANNER_EVT_UUID_INVALID,
} scanner_event_id_t;

typedef struct 
{
    scanner_event_id_t id;
    char data[36];
} scanner_event_t;

typedef void (*scanner_app_cb)(scanner_event_t* evt);

scanner_status_t scanner_app_init(scanner_app_cb user_cb);

void scanner_app_start(void);

void scanner_app_stop(void);

void scanner_app_deinit(void);

void scanner_app_get_data(char* data);

scanner_status_t scanner_app_trigger(void);

scanner_status_t scanner_app_sleep(void);

#endif /* COMPONENTS_SCANNER_SCANNER_APP */
