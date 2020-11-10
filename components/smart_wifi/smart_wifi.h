#ifndef COMPONENTS_SMART_WIFI_SMART_WIFI
#define COMPONENTS_SMART_WIFI_SMART_WIFI

typedef enum {
    EVT_CONNECT,
    EVT_DISCONNECT,
    EVT_SMART_CONFIG_START,
    EVT_SMART_CONFIG_SUCCES,
    EVT_SMART_CONFIG_FAIL,
    EVT_FAIL,
} smart_wifi_event_id_t;

typedef struct smart_wifi_event {
    smart_wifi_event_id_t id;
} smart_wifi_event_t;

typedef void (*smart_wifi_app_cb)(smart_wifi_event_t* evt);

void initialise_wifi(smart_wifi_app_cb app_cb);

#endif /* COMPONENTS_SMART_WIFI_SMART_WIFI */
